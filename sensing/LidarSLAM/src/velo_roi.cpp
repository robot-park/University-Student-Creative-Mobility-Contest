// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following papers:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
//   T. Shan and B. Englot. LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain
//      IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). October 2018.
#include <ros/ros.h>
#include "gigacha_p/utility.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <math.h>

///########################################추가 ##################################################################
#include "gigacha_p/velocity.h"
#include "gigacha_p/result_length.h"
#define w_lengh 6
#define radius 3 
#define MINI_VELOCITY 5
#define MAX_VELOCITY 18
//###############################################################################################################
#define GRemoval_TH 10
#define GRemoval_angle -1.0

class ImageProjection{
private:

    ros::NodeHandle nh;

    ros::Subscriber subLaserCloud;
    
    ros::Publisher pubFullCloud;
    ros::Publisher pubFullInfoCloud;

    ros::Publisher pubGroundCloud;
    ros::Publisher pubSegmentedCloud;
    ros::Publisher pubSegmentedCloudPure;
    ros::Publisher pubtmpCloud;
    ros::Publisher pubSegmentedCloudInfo;
    ros::Publisher pubClusterCloud;
    ros::Publisher pubOutlierCloud;
    ros::Publisher pubMarkerArray;
    //####################추가###############################
    ros::Subscriber subVelocity;
    ros::Publisher pub_h_length;
    ros::Publisher pubSomeCloud;
    pcl::PointCloud<PointType>::Ptr someCloud;

    pcl::PointCloud<PointType>::Ptr laserCloudIn;
    // pcl::PointCloud<PointXYZIR>::Ptr laserCloudInRing;

    pcl::PointCloud<PointType>::Ptr fullCloud; // projected velodyne raw cloud, but saved in the form of 1-D matrix
    pcl::PointCloud<PointType>::Ptr fullInfoCloud; // same as fullCloud, but with intensity - range

    pcl::PointCloud<PointType>::Ptr groundCloud;
    pcl::PointCloud<PointType>::Ptr segmentedCloud;
    pcl::PointCloud<PointType>::Ptr segmentedCloudPure;
    pcl::PointCloud<PointType>::Ptr ClusterCloud;
    std::vector <pcl::PointCloud<PointType>::Ptr> ClusterClouds;
    pcl::PointCloud<PointType>::Ptr tmpCloud;
    pcl::PointCloud<PointType>::Ptr outlierCloud;
    


    sensor_msgs::PointCloud2 output; 

    //visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray markerarray;

    PointType nanPoint; // fill in fullCloud at each iteration

    cv::Mat rangeMat; // range matrix for range image
    cv::Mat labelMat; // label matrix for segmentaiton marking
    cv::Mat groundMat; // ground matrix for ground cloud marking
    cv::Mat cloudMat;
    int labelCount;
    //#########추가#############
    float h_length_v;

    float startOrientation;
    float endOrientation;

    std_msgs::Header cloudHeader;

    std::vector<std::pair<int8_t, int8_t> > neighborIterator; // neighbor iterator for segmentaiton process
    std::vector<std::pair<int8_t, int8_t> > neighbor2Iterator; // neighbor2 iterator for doubling channel
    uint16_t *allPushedIndX; // array for tracking points of a segmented object
    uint16_t *allPushedIndY;

    uint16_t *queueIndX; // array for breadth-first search process of segmentation, for speed
    uint16_t *queueIndY;

public:
    ImageProjection():
        nh("~"){

        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &ImageProjection::cloudHandler, this);
        //##########추가###########
        subVelocity = nh.subscribe("/car_velocity", 10, &ImageProjection::velocityHandler, this);
        // subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/filtered_channel", 1, &ImageProjection::cloudHandler, this);

        pubFullCloud = nh.advertise<sensor_msgs::PointCloud2> ("/full_cloud_projected", 1);
        pubFullInfoCloud = nh.advertise<sensor_msgs::PointCloud2> ("/full_cloud_info", 1);

        pubGroundCloud = nh.advertise<sensor_msgs::PointCloud2> ("/ground_cloud", 1);
        pubSegmentedCloud = nh.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud", 1);
        pubSegmentedCloudPure = nh.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud_pure", 1);
        pubtmpCloud = nh.advertise<sensor_msgs::PointCloud2> ("/tmpcloud", 1);
        pubOutlierCloud = nh.advertise<sensor_msgs::PointCloud2> ("/outlier_cloud", 1);
        pubClusterCloud = nh.advertise<sensor_msgs::PointCloud2> ("/ClusterCloud", 1);
        pubMarkerArray = nh.advertise<visualization_msgs::MarkerArray>("markers", 10);
        //#######추가#####
        pubSomeCloud = nh.advertise<sensor_msgs::PointCloud2> ("/some_cloud",1);
        pub_h_length = nh.advertise<gigacha_p::result_length>("/h_length", 10);


        nanPoint.x = std::numeric_limits<float>::quiet_NaN();
        nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        nanPoint.z = std::numeric_limits<float>::quiet_NaN();
        nanPoint.intensity = -1;

        allocateMemory();
        resetParameters();
    }

    void allocateMemory(){

        laserCloudIn.reset(new pcl::PointCloud<PointType>());
        // laserCloudInRing.reset(new pcl::PointCloud<PointXYZIR>());

        fullCloud.reset(new pcl::PointCloud<PointType>());
        fullInfoCloud.reset(new pcl::PointCloud<PointType>());

        //######추가###########
        someCloud.reset(new pcl::PointCloud<PointType>());

        groundCloud.reset(new pcl::PointCloud<PointType>());
        segmentedCloud.reset(new pcl::PointCloud<PointType>());
        segmentedCloudPure.reset(new pcl::PointCloud<PointType>());
        ClusterCloud.reset(new pcl::PointCloud<PointType>());
        tmpCloud.reset(new pcl::PointCloud<PointType>());
        outlierCloud.reset(new pcl::PointCloud<PointType>());
        fullCloud->points.resize(N_SCAN*Horizon_SCAN);
        //########추가################
        someCloud->points.resize(N_SCAN*Horizon_SCAN);
        fullInfoCloud->points.resize(N_SCAN*Horizon_SCAN);
        

        std::pair<int8_t, int8_t> neighbor;
        neighbor.first = -1; neighbor.second =  0; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second =  1; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second = -1; neighborIterator.push_back(neighbor);
        neighbor.first =  1; neighbor.second =  0; neighborIterator.push_back(neighbor);

        std::pair<int8_t, int8_t> neighbor2;
        neighbor2.first = -1; neighbor2.second = 1; neighbor2Iterator.push_back(neighbor2);
        neighbor2.first = -1; neighbor2.second = -1; neighbor2Iterator.push_back(neighbor2);
        neighbor2.first = 1; neighbor2.second = 1; neighbor2Iterator.push_back(neighbor2);
        neighbor2.first = 1; neighbor2.second = -1; neighbor2Iterator.push_back(neighbor2);

        allPushedIndX = new uint16_t[N_SCAN*Horizon_SCAN];
        allPushedIndY = new uint16_t[N_SCAN*Horizon_SCAN];

        queueIndX = new uint16_t[N_SCAN*Horizon_SCAN];
        queueIndY = new uint16_t[N_SCAN*Horizon_SCAN];
    }

    void resetParameters(){
        laserCloudIn->clear();
        groundCloud->clear();
        segmentedCloud->clear();
        segmentedCloudPure->clear();
        tmpCloud->clear();
        outlierCloud->clear();
        ClusterCloud->clear();
        ClusterClouds.clear();
        //######추가###########
        someCloud->clear();

        markerarray.markers.clear();

        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
        groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
        labelMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));
        cloudMat = cv::Mat(31, Horizon_SCAN, CV_32F, cv::Scalar::all(0));

        labelCount = 1;

        std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
        //#####추가####
        //std::fill(someCloud->points.begin(), someCloud->points.end(), nanPoint);

        std::fill(fullInfoCloud->points.begin(), fullInfoCloud->points.end(), nanPoint);
    }

    ~ImageProjection(){}

    void copyPointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

        cloudHeader = laserCloudMsg->header;
        // cloudHeader.stamp = ros::Time::now(); // Ouster lidar users may need to uncomment this line
        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
        // Remove Nan points
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);

    }

    void velocityHandler(const gigacha_p::velocity& vel_msg){
        if(vel_msg.velocity <=MINI_VELOCITY) h_length_v = 2;// 볼 면적의 마지노선을 설정
        // 이건 알잘딱으로 수정하면됨 -> 원하는 방향대로 속도를 바탕으로 어디까지 볼 것인가를 지정
        else if(vel_msg.velocity >= MAX_VELOCITY ) h_length_v = 18;
        else h_length_v =  vel_msg.velocity  ; //대충 속도의 1배만큼 볼 것이다. (단위는 m)
        
        gigacha_p::result_length result_length_msg;
        result_length_msg.h_length = h_length_v;
        cout << h_length_v <<endl;

        pub_h_length.publish(result_length_msg);
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

        // 1. Convert ros message to pcl point cloud
        copyPointCloud(laserCloudMsg);
        // 2. Start and end angle of a scan
        // 3. Range image projection
        projectPointCloud();
        // 4. Mark ground points
        groundRemoval();
        // 5. Point cloud segmentation
        cloudSegmentation();
        //clustering();
        //filteringObject();
        // 6. Publish all clouds
        publishCloud();
        // 7. Reset parameters for next iteration
        resetParameters();
    }



    void projectPointCloud(){
        // range image projection
        float verticalAngle, horizonAngle, range;
        size_t rowIdn, columnIdn, index, cloudSize; 
        PointType thisPoint;

        cloudSize = laserCloudIn->points.size();

        for (size_t i = 0; i < cloudSize; ++i){

            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            // find the row and column index in the iamge for this point
            // if (useCloudRing == true){
            //     rowIdn = laserCloudInRing->points[i].ring;
            // }
            verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
            rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
            
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

            columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
            if (columnIdn >= Horizon_SCAN)
                columnIdn -= Horizon_SCAN;

            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
            if (range < sensorMinimumRange)
                continue;
            
            rangeMat.at<float>(rowIdn, columnIdn) = range;

            thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

            index = columnIdn  + rowIdn * Horizon_SCAN;
            fullCloud->points[index] = thisPoint;
            fullInfoCloud->points[index] = thisPoint;
            fullInfoCloud->points[index].intensity = range; // the corresponding range of a point is saved as "intensity"
            //######추가##########
            if(set_ROi(thisPoint)) {
                cout << "hhhhhh\n"; 
                someCloud->points.push_back(thisPoint);
            }
            
        }
        

        for (int u = 0; u < N_SCAN - 1; u++) {
            for (int v = 0; v < Horizon_SCAN; v++) {
                rowIdn = 14 - u;
                columnIdn = v;
                cloudMat.at<float>(rowIdn * 2 + 1, v) = (rangeMat.at<float>(rowIdn*2, columnIdn) + rangeMat.at<float>(rowIdn*2 + 2, columnIdn)) / 2;
            }
        }
    }

    //################추가###########
    bool set_ROi(const PointType& point){
        float cir_x = point.x - h_length_v;

        if((abs(point.y) < w_lengh/2 && point.x < h_length_v - radius ) ||
                    (abs(point.y) < sqrt(radius*radius - cir_x*cir_x) && point.x < sqrt(radius*radius - point.y*point.y) && point.x > h_length_v - radius ))
            return true;
        else 
            return false;
    }

    void groundRemoval() {
        size_t lowerInd, upperInd;
        float diffX, diffY, diffZ, angle;
        // groundMat
        // -1, no valid info to check if ground of not
        //  0, initial value, after validation, means not ground
        //  1, ground
        for (size_t j = 0; j < Horizon_SCAN; j++) {
            for (size_t i = 0; i < groundScanInd; ++i) {

                lowerInd = j + (i)*Horizon_SCAN;
                upperInd = j + (i + 1) * Horizon_SCAN;

                if (fullCloud->points[lowerInd].intensity == -1 ||
                    fullCloud->points[upperInd].intensity == -1) {
                    // no info to check, invalid points
                    groundMat.at<int8_t>(i, j) = -1;
                    continue;
                }

                diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
                diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
                diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;

                angle = atan2(diffZ, sqrt(diffX * diffX + diffY * diffY)) * 180 / M_PI;

                if ((abs(angle) <= GRemoval_angle && fullCloud->points[lowerInd].z < GRemoval_TH) || (isnan(angle) && fullCloud->points[lowerInd].z < GRemoval_TH)) {
                    groundMat.at<int8_t>(i, j) = 1;
                    groundMat.at<int8_t>(i + 1, j) = 1;
                }
                else {
                    upperInd = j + (i + 2) * Horizon_SCAN;
                    if (fullCloud->points[lowerInd].intensity == -1 ||
                        fullCloud->points[upperInd].intensity == -1) {
                        groundMat.at<int8_t>(i, j) = -1;
                        continue;
                    }

                    diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
                    diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
                    diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;

                    angle = angle = atan2(diffZ, sqrt(diffX * diffX + diffY * diffY)) * 180 / M_PI;

                    if ((abs(angle) <= GRemoval_angle && fullCloud->points[lowerInd].z < GRemoval_TH) || (isnan(angle) && fullCloud->points[lowerInd].z < GRemoval_TH)) {
                        groundMat.at<int8_t>(i, j) = 1;
                        groundMat.at<int8_t>(i + 2, j) = 1;
                    }
                }
            }
        }
        // extract ground cloud (groundMat == 1)
        // mark entry that doesn't need to label (ground and invalid point) for segmentation
        // note that ground remove is from 0~N_SCAN-1, need rangeMat for mark label matrix for the 16th scan
        for (size_t i = 0; i < N_SCAN; ++i) {
            for (size_t j = 0; j < Horizon_SCAN; ++j) {
                if (groundMat.at<int8_t>(i, j) == 1 || rangeMat.at<float>(i, j) == FLT_MAX) {
                    labelMat.at<int>(i, j) = -1;
                }
            }
        }
        if (pubGroundCloud.getNumSubscribers() != 0) {
            for (size_t i = 0; i <= groundScanInd; ++i) {
                for (size_t j = 0; j < Horizon_SCAN; ++j) {
                    if (groundMat.at<int8_t>(i, j) == 1)
                        groundCloud->push_back(fullCloud->points[j + i * Horizon_SCAN]);
                }
            }
        }
    }

    void cloudSegmentation(){
        // segmentation process
        for (size_t i = 0; i < N_SCAN; ++i)
            for (size_t j = 0; j < Horizon_SCAN; ++j)
                if (labelMat.at<int>(i,j) == 0)
                    labelComponents(i, j);

        int sizeOfSegCloud = 0;
        // extract segmented cloud for lidar odometry
        for (size_t i = 0; i < N_SCAN; ++i) {


            for (size_t j = 0; j < Horizon_SCAN; ++j) {
                if (labelMat.at<int>(i,j) > 0 || groundMat.at<int8_t>(i,j) == 1){
                    // outliers that will not be used for optimization (always continue)
                    if (labelMat.at<int>(i,j) == 999999){
                        if (i > groundScanInd && j % 5 == 0){
                            outlierCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                            continue;
                        }else{
                            continue;
                        }
                    }
                    // majority of ground points are skipped
                    if (groundMat.at<int8_t>(i,j) == 1){
                        if (j%5!=0 && j>5 && j<Horizon_SCAN-5)
                            continue;
                    }
                    segmentedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                    // size of seg cloud
                    ++sizeOfSegCloud;
                }
            }

        }
        
        // extract segmented cloud for visualization
        // rviz에서 segmentedCloudPure pointCloud를 요청하지 않는 이상 upsampling을 진행하지 않음
        if (pubSegmentedCloudPure.getNumSubscribers() != 0){
            for (size_t i = 0; i < N_SCAN; ++i){
                for (size_t j = 0; j < Horizon_SCAN; ++j){
                    if (labelMat.at<int>(i,j) > 0 && labelMat.at<int>(i,j) != 999999){
                        segmentedCloudPure->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                        segmentedCloudPure->points.back().intensity = labelMat.at<int>(i,j);
                        //upsampling(i,j);
                    }
                }
            }
            if (pubClusterCloud.getNumSubscribers() != 0){
                clustering();
                //filteringObject();
            }
        }
        
         tmpCloud->width = (int) tmpCloud->points.size();
         tmpCloud->height = 1;
         tmpCloud->is_dense = true;
    }

    void labelComponents(int row, int col){
        // use std::queue std::vector std::deque will slow the program down greatly
        float d1, d2, dist, angle, alpha;
        float minimun_dist_x = 0.5;
        float minimun_dist_y = 0.015;
        int fromIndX, fromIndY, thisIndX, thisIndY; 
        bool lineCountFlag[N_SCAN] = {false};

        queueIndX[0] = row;
        queueIndY[0] = col;
        int queueSize = 1;
        int queueStartInd = 0;
        int queueEndInd = 1;

        allPushedIndX[0] = row;
        allPushedIndY[0] = col;
        int allPushedIndSize = 1;
        
        while(queueSize > 0){
            // Pop point
            fromIndX = queueIndX[queueStartInd];
            fromIndY = queueIndY[queueStartInd];
            --queueSize;
            ++queueStartInd;
            // Mark popped point
            labelMat.at<int>(fromIndX, fromIndY) = labelCount;
            // Loop through all the neighboring grids of popped grid
            for (auto iter = neighborIterator.begin(); iter != neighborIterator.end(); ++iter){
                // new index
                thisIndX = fromIndX + (*iter).first;
                thisIndY = fromIndY + (*iter).second;
                // index should be within the boundary
                if (thisIndX < 0 || thisIndX >= N_SCAN)
                    continue;
                // at range image margin (left or right side)
                if (thisIndY < 0)
                    thisIndY = Horizon_SCAN - 1;
                if (thisIndY >= Horizon_SCAN)
                    thisIndY = 0;
                // prevent infinite loop (caused by put already examined point back)
                if (labelMat.at<int>(thisIndX, thisIndY) != 0)
                    continue;

                d1 = std::max(rangeMat.at<float>(fromIndX, fromIndY), 
                              rangeMat.at<float>(thisIndX, thisIndY));
                d2 = std::min(rangeMat.at<float>(fromIndX, fromIndY), 
                              rangeMat.at<float>(thisIndX, thisIndY));

                // if ((*iter).first == 0)
                //     alpha = segmentAlphaX;
                // else
                //     alpha = segmentAlphaY;

                // angle = atan2(d2*sin(alpha), (d1 -d2*cos(alpha)));

                
                if (fromIndX = thisIndX) {
                    if ((d1 - d2) < minimun_dist_x) { // (이거 설명 위 아래 바뀜)minimun_dist_x는 엄청 작게 30m 밖에서 점과 점사이 수평거리는 1센치가 조금 넘으니 적어도 1.1cm으로 해놓으면 될 듯

                        queueIndX[queueEndInd] = thisIndX;
                        queueIndY[queueEndInd] = thisIndY;
                        ++queueSize;
                        ++queueEndInd;

                        labelMat.at<int>(thisIndX, thisIndY) = labelCount;
                        lineCountFlag[thisIndX] = true; // 이 부분을 봐야겠다 이게 위 아래를 보는 부분이니까 
                        //thisIndX면 i값을 의미하는 것임.
                        allPushedIndX[allPushedIndSize] = thisIndX;
                        allPushedIndY[allPushedIndSize] = thisIndY;
                        ++allPushedIndSize;
                    }
                }


                if (fromIndY = thisIndY) {        // minimun_dist_y 는 저거보다는 조금 더 크게. maybe like 40cm?
                    if ((d1 - d2) < minimun_dist_y) {

                        queueIndX[queueEndInd] = thisIndX;
                        queueIndY[queueEndInd] = thisIndY;
                        ++queueSize;
                        ++queueEndInd;

                        labelMat.at<int>(thisIndX, thisIndY) = labelCount;
                        lineCountFlag[thisIndX] = true; // 우린 i좌표만 보면 되기는 한다. 이건 매번 초기화

                        allPushedIndX[allPushedIndSize] = thisIndX;
                        allPushedIndY[allPushedIndSize] = thisIndY;
                        ++allPushedIndSize;
                        // 바로 위 3개의 코드같은 경우에는 allPushedIndX에 allPushedIndSize번째에 지금의 i좌표인 thisIndX가 해당된다.
                    }
                }
            }
        }

        // check if this segment is valid
        bool feasibleSegment = false;
        if (allPushedIndSize >= 30)
            feasibleSegment = true;
        else if (allPushedIndSize >= segmentValidPointNum){
            int lineCount = 0;
            for (size_t i = 0; i < N_SCAN; ++i)
                if (lineCountFlag[i] == true)
                    ++lineCount;
            if (lineCount >= segmentValidLineNum)
                feasibleSegment = true;            
        }
        // segment is valid, mark these points
        if (feasibleSegment == true){
            ++labelCount;
        }else{ // segment is invalid, mark these points
            for (size_t i = 0; i < allPushedIndSize; ++i){
                labelMat.at<int>(allPushedIndX[i], allPushedIndY[i]) = 999999;
            }
        }
    }

    void upsampling(int row, int col) { //이거 cloud segmentation에 집어 넣어야한다.

        int thisIndX, thisIndY;
        int fromIndX = 2 * row - 1;
        int fromIndY = col;
        float w_sum = 0;
        float tmp_x = 0;
        float tmp_y = 0;
        float tmp_z = 0;
        float tmp_I = 0;
        float d;
        for (auto iter = neighbor2Iterator.begin(); iter != neighbor2Iterator.end(); ++iter) {
            thisIndX = fromIndX + (*iter).first;
            thisIndY = fromIndY + (*iter).second;

            if (thisIndX < 0 || thisIndX >= N_SCAN * 2) // thisIndX가 위로 올라가버리면 그러게 그냥 패스 해버려도 되고 애초에 필요가 없기도 하겠다 근데
                continue;

            if (thisIndY < 0)
                thisIndY = Horizon_SCAN - 1;

            if (thisIndY >= Horizon_SCAN) //this포인트를 걸러내야 함
                thisIndY = 0;

            if (labelMat.at<int>(thisIndX/2, thisIndY) != labelMat.at<int>(row,col))
                continue;

            //if (labelMat.at<int>(thisIndX, thisIndY) != 0) //이 조건은 필요 없음 우린 뭐든간에 다 볼거라서
            //    continue; // cloudMat은 31*1800짜리 채널이다 명심하도록.
            d = abs(cloudMat.at<float>(fromIndX, fromIndY) - cloudMat.at<float>(thisIndX, thisIndY));
            tmp_x += exp(-d) * fullCloud->points[thisIndY + thisIndX/2 * 1800].x;
            tmp_y += exp(-d) * fullCloud->points[thisIndY + thisIndX/2 * 1800].y;
            tmp_z += exp(-d) * fullCloud->points[thisIndY + thisIndX/2 * 1800].z;
            w_sum += exp(-d);
        }
        tmp_x = tmp_x/w_sum;
        tmp_y = tmp_y/w_sum;
        tmp_z = tmp_z/w_sum;

        //std::cout << tmp_z << std::endl;

        pcl::PointXYZI point;
        point.x = tmp_x;
        point.y = tmp_y;
        point.z = tmp_z;

        segmentedCloudPure->push_back(point);
        segmentedCloudPure->points.back().intensity = labelMat.at<int>(row,col);

    }
    
    void clustering(){
        
        // 확인용 msg -> NaN 잘 제거 되었는지 확인, kd-tree에서는 NaN 활용 불가
        //cout <<"1: "<< segmentedCloudPure->points.size()<<endl;
        segmentedCloudPure->is_dense=false; // 직접 만든 pointCloud에 대해서는 flase를 주어야함
        std::vector<int> indices;
        //std::cout << "NaN removed\n";
        pcl::removeNaNFromPointCloud(*segmentedCloudPure,*segmentedCloudPure,indices);
        //cout <<"2: "<<segmentedCloudPure->points.size()<<endl<<endl<<endl;
        ClusterClouds.clear();

        double clusterTolerance = 0.06;
        int minSize = 30;
        int maxSize = 5000;
        int clusterId = 0;
        if (segmentedCloudPure->points.size() > 0){
            pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
            tree->setInputCloud(segmentedCloudPure);
            std::vector<pcl::PointIndices> clusterIndices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
            ec.setClusterTolerance(clusterTolerance);
            ec.setMinClusterSize(minSize);
            ec.setMaxClusterSize(maxSize);
            ec.setSearchMethod(tree);
            ec.setInputCloud(segmentedCloudPure);
            ec.extract(clusterIndices);

            int j=0;
            for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin (); it != clusterIndices.end (); ++it)
            {
                for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                {
                    pcl::PointXYZI pt = segmentedCloudPure->points[*pit];
                    pcl::PointXYZI pt2;

                    pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
                    pt2.intensity = (float)(j + 1);
                    ClusterCloud->points.push_back(pt2);
                    ClusterCloud->points.back().intensity = pt2.intensity;


                }

                ClusterCloud->width = ClusterCloud->points.size();
                ClusterCloud->height = 1;
                ClusterCloud->is_dense = false;

                ClusterClouds.push_back(ClusterCloud);
                filteringObject(clusterId,it);
                
                clusterId++;
                j++;
            }
            std::cout <<"clustering: "<< j<<endl;
        }
        
      
    }
        
    
    void filteringObject(int Id , vector<pcl::PointIndices>::const_iterator it){
        //visual marker
        visualization_msgs::Marker marker;

        pcl::PointXYZI minPoint, maxPoint;
        pcl::getMinMax3D(*ClusterClouds.back(), minPoint, maxPoint);

        double rangeX = maxPoint.x - minPoint.x;
        double rangeY = maxPoint.y - minPoint.y;
        double rangeZ = maxPoint.z - minPoint.z;
        double diagonal_distance = sqrt((int)(rangeX)*(rangeX)+(int)(rangeY)*(rangeY)+(int)(rangeZ)*(rangeZ));
        //std::cout << diagonal_distance << endl;
        if(diagonal_distance < 4.0){
            marker.header.frame_id = "base_link";
            marker.header.stamp = ros::Time::now();
            marker.id = Id;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose.position.x = (maxPoint.x + minPoint.x) / 2;
            marker.pose.position.y = (maxPoint.y + minPoint.y) / 2;
            marker.pose.position.z = (maxPoint.z + minPoint.z) / 2;

            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = fabs(maxPoint.x - minPoint.x);
            marker.scale.y = fabs(maxPoint.y - minPoint.y);
            marker.scale.z = fabs(maxPoint.z - minPoint.z);
            // marker.scale.x = 0.1;
            // marker.scale.y = 0.1;
            // marker.scale.z = 0.1;

            //srand((unsigned int)time(NULL));

            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.color.a = 0.7;

            marker.lifetime = ros::Duration(0.1);
    
            markerarray.markers.push_back(marker);
        }

        
    }


    void publishCloud(){
        // 1. Publish Seg Cloud Info
        // 2. Publish clouds
        sensor_msgs::PointCloud2 laserCloudTemp;

        pcl::toROSMsg(*outlierCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "base_link";
        pubOutlierCloud.publish(laserCloudTemp);
        // segmented cloud with ground
        pcl::toROSMsg(*segmentedCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "base_link";
        pubSegmentedCloud.publish(laserCloudTemp);
        // projected full cloud
        if (pubFullCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*fullCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubFullCloud.publish(laserCloudTemp);
        }
        // original dense ground cloud
        if (pubGroundCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*groundCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubGroundCloud.publish(laserCloudTemp);
        }
        // segmented cloud without ground
        if (pubSegmentedCloudPure.getNumSubscribers() != 0){
            pcl::toROSMsg(*segmentedCloudPure, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubSegmentedCloudPure.publish(laserCloudTemp);

            if (pubSegmentedCloudPure.getNumSubscribers() !=0){
                pcl::toROSMsg(*ClusterCloud, laserCloudTemp);
                laserCloudTemp.header.stamp = cloudHeader.stamp;
                laserCloudTemp.header.frame_id = "base_link";
                pubClusterCloud.publish(laserCloudTemp);
        }    
        }
        if (pubtmpCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*tmpCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubtmpCloud.publish(laserCloudTemp);
        }
        // projected full cloud info
        if (pubFullInfoCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*fullInfoCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubFullInfoCloud.publish(laserCloudTemp);
        }
        //#########추가#######################
        if (pubSomeCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*someCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubSomeCloud.publish(laserCloudTemp);
        }
        if(pubMarkerArray.getNumSubscribers() !=0) pubMarkerArray.publish(markerarray);

        
    }
};




int main(int argc, char** argv){

    ros::init(argc, argv, "lego_loam");
    
    ImageProjection IP;


    ROS_INFO("\033[1;32m---->\033[0m Image Projection Started.");

    ros::spin();
    return 0;
	}