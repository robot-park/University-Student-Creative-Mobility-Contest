#include "utility.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/Detection2D.h>
#include <pcl/kdtree/kdtree_flann.h>
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
#include <chrono>
#include <tuple>
#include <queue>


#define GRemoval_TH -0.25
#define GRemoval_angle 25.0

class ImageProjection{
private:

    ros::NodeHandle nh;
    // message_filters::Subscriber<sensor_msgs::PointCloud2> subLaserCloud;
    // message_filters::Subscriber<vision_msgs::Detection2DArray> subDetection;
    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, vision_msgs::Detection2DArray> MySyncPolicy;
    // typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    // boost::shared_ptr<Sync> sync;

    ros::Subscriber subLaserCloud;
    
    ros::Publisher pubFullCloud;
    ros::Publisher pubFullInfoCloud;
    ros::Publisher pubupsampledCloud;

    ros::Publisher pubGroundCloud;
    ros::Publisher pubSegmentedCloud;
    ros::Publisher pubSegmentedCloudPure;
    ros::Publisher pubtmpCloud;
    ros::Publisher pubSegmentedCloudInfo;
    ros::Publisher pubClusterCloud;
    ros::Publisher pubOutlierCloud;
    ros::Publisher pubMarkerArray;


    pcl::PointCloud<PointType>::Ptr laserCloudIn;
    // pcl::PointCloud<PointXYZIR>::Ptr laserCloudInRing;

    pcl::PointCloud<PointType>::Ptr fullCloud; // projected velodyne raw cloud, but saved in the form of 1-D matrix
    pcl::PointCloud<PointType>::Ptr fullInfoCloud; // same as fullCloud, but with intensity - range
    pcl::PointCloud<PointType>::Ptr upsampledCloud;

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
    visualization_msgs::Marker marker;

    PointType nanPoint; // fill in fullCloud at each iteration

    cv::Mat rangeMat; // range matrix for range image
    cv::Mat labelMat; // label matrix for segmentaiton marking
    cv::Mat groundMat; // ground matrix for ground cloud marking
    cv::Mat cloudMat;
    cv::Mat rangeMat_2;
    cv::Mat upsampleMat; // upsampleMat for upsampling to 61 channel (not 64 channel)
    cv::Mat resultMat;
    cv::Mat visited;
    // cv::Mat mask;

    int labelCount;
    int markercnt = 0;

    float startOrientation;
    float endOrientation;

    std_msgs::Header cloudHeader;

    std::vector<std::pair<int8_t, int8_t> > neighborIterator; // neighbor iterator for segmentaiton process
    std::vector<std::pair<int8_t, int8_t> > neighbor2Iterator; // neighbor2 iterator for doubling channel

    std::vector<std::pair<int8_t, int8_t> > neighbor3Iterator;
    std::vector<std::pair<int8_t, int8_t> > neighbor4Iterator;
    std::vector<std::pair<int8_t, int8_t> > neighbor5Iterator;

    uint16_t *allPushedIndX; // array for tracking points of a segmented object
    uint16_t *allPushedIndY;

    uint16_t *queueIndX; // array for breadth-first search process of segmentation, for speed
    uint16_t *queueIndY;

public:
    ImageProjection():
        nh("~"){
            
        // subLaserCloud.subscribe(nh, "/velodyne_points", 1);
        // subDetection.subscribe(nh, "/jaejun/bbox_info", 1);
        // sync.reset(new Sync(MySyncPolicy(10), subLaserCloud, subDetection));
        // sync->registerCallback(boost::bind(&ImageProjection::cloudHandler, this, _1, _2));
        
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &ImageProjection::cloudHandler, this);
        // subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/filtered_channel", 1, &ImageProjection::cloudHandler, this);
        // message_filters::Subscriber<sensor_msgs::PointCloud2> velo_sub(nh, "/velodyne_points", 1);
        // message_filters::Subscriber<sensor_msgs::PointCloud2> yolo_sub(nh, "/pc2", 1);
        // message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> sync(velo_sub, yolo_sub, 10);
        // sync.registerCallback(boost::bind(&ImageProjection::cloudHandler, _1, _2, this));

        pubFullCloud = nh.advertise<sensor_msgs::PointCloud2> ("/full_cloud_projected", 1);
        pubFullInfoCloud = nh.advertise<sensor_msgs::PointCloud2> ("/full_cloud_info", 1);

        pubGroundCloud = nh.advertise<sensor_msgs::PointCloud2> ("/ground_cloud", 1);
        pubSegmentedCloud = nh.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud", 1);
        pubSegmentedCloudPure = nh.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud_pure", 1);
        pubtmpCloud = nh.advertise<sensor_msgs::PointCloud2> ("/tmpcloud", 1);
        pubOutlierCloud = nh.advertise<sensor_msgs::PointCloud2> ("/outlier_cloud", 1);
        pubClusterCloud = nh.advertise<sensor_msgs::PointCloud2> ("/ClusterCloud", 1);
        pubMarkerArray = nh.advertise<visualization_msgs::MarkerArray>("/markers", 10);
        pubupsampledCloud = nh.advertise<sensor_msgs::PointCloud2> ("/upsampled_cloud", 1);


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
        upsampledCloud.reset(new pcl::PointCloud<PointType>());

        groundCloud.reset(new pcl::PointCloud<PointType>());
        segmentedCloud.reset(new pcl::PointCloud<PointType>());
        segmentedCloudPure.reset(new pcl::PointCloud<PointType>());
        ClusterCloud.reset(new pcl::PointCloud<PointType>());
        tmpCloud.reset(new pcl::PointCloud<PointType>());
        outlierCloud.reset(new pcl::PointCloud<PointType>());
        fullCloud->points.resize(N_SCAN*Horizon_SCAN);
        fullInfoCloud->points.resize(N_SCAN*Horizon_SCAN);
        // upsampledCloud->points.resize(61*Horizon_SCAN);
        

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
        upsampledCloud->clear();
        
        markerarray.markers.clear();

        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
        groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
        labelMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));
        cloudMat = cv::Mat(31, Horizon_SCAN, CV_32F, cv::Scalar::all(0));
        rangeMat_2 = cv::Mat(61, Horizon_SCAN, CV_32F, cv::Scalar::all(0));
        upsampleMat = cv::Mat(61, Horizon_SCAN, CV_32F, cv::Scalar::all(0));
        visited = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(0));
        // mask = cv::Mat(480,640, CV_8UC1, cv::Scalar::all(0));
        labelCount = 1;

        std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
        std::fill(fullInfoCloud->points.begin(), fullInfoCloud->points.end(), nanPoint);
        markercnt = 0;
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
    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){//},const vision_msgs::Detection2DArray::ConstPtr& bboxMsg){
        std::chrono::system_clock::time_point starttime = std::chrono::system_clock::now();
        // 1. Convert ros message to pcl point cloud
        copyPointCloud(laserCloudMsg);
        // detection(bboxMsg);
        // 2. Start and end angle of a scan
        // 3. Range image projection
        projectPointCloud();
        // 4. Mark ground points
        groundRemoval();
        // 5. Point cloud segmentation
        cloudSegmentation();

        clustering();
        filteringObject();
        // 6. Publish all clouds
        publishCloud();
        // 7. Reset parameters for next iteration
        resetParameters();
        std::chrono::system_clock::time_point endtime = std::chrono::system_clock::now();
        std::chrono::milliseconds mill=std::chrono::duration_cast<chrono::milliseconds> (endtime - starttime);
        // std::cout << 1000 / mill.count() << "Hz" << std::endl;

        // std::cout << "done" << std::endl;
    }
    
    // void detection(const vision_msgs::Detection2DArrayConstPtr& detection_msg){
    //     // cv::Mat mask(480,640,CV_8UC1, cv::Scalar(0))
    //     std::cout << "running" << std::endl;
    //     for (int i =0; i<detection_msg->detections.size(); i++){
    //         vision_msgs::Detection2D detection = detection_msg->detections[i];
    //         cv::Rect bbox(detection.bbox.center.x - detection.bbox.size_x/2,detection.bbox.center.y - detection.bbox.size_y/2,detection.bbox.size_x, detection.bbox.size_y);
    //         cv::Mat roi(mask, bbox);
    //         roi.setTo(cv::Scalar(1));
    //     }
    // }

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
            rangeMat_2.at<float>(rowIdn * 4, columnIdn) = range;
            upsampleMat.at<float>(rowIdn*4, columnIdn) = range;
            

            thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

            index = columnIdn  + rowIdn * Horizon_SCAN;
            fullCloud->points[index] = thisPoint;
            fullInfoCloud->points[index] = thisPoint;
            fullInfoCloud->points[index].intensity = range; // the corresponding range of a point is saved as "intensity"
        }
        
        //for range min matrix
        // for (int u = 0; u < 61; u++) {
        //     for (int v = 1 ; v < Horizon_SCAN-1; v++) {
            
        //         if (std::min<float>({rangeMat.at<float>(u/4 + 1,v-1), rangeMat.at<float>(u/4 + 1,v), rangeMat.at<float>(u/4 + 1,v+1),rangeMat.at<float>(u/4,v-1),rangeMat.at<float>(u/4,v),rangeMat.at<float>(u/4,v+1)}) > 0.5 && std::min<float>({rangeMat.at<float>(u/4 + 1,v-1), rangeMat.at<float>(u/4 + 1,v), rangeMat.at<float>(u/4 + 1,v+1),rangeMat.at<float>(u/4,v-1),rangeMat.at<float>(u/4,v),rangeMat.at<float>(u/4,v+1)}) < 100);
        //             rangeMat_2.at<float>(u,v) = std::min<float>({rangeMat.at<float>(u/4 + 1,v-1), rangeMat.at<float>(u/4 + 1,v), rangeMat.at<float>(u/4 + 1,v+1),rangeMat.at<float>(u/4,v-1),rangeMat.at<float>(u/4,v),rangeMat.at<float>(u/4,v+1)});
                
        //     }
        // }
    }


    void groundRemoval() {
        size_t lowerInd, upperInd;
        float diffX, diffY, diffZ, angle;
        for (size_t j = 0; j < Horizon_SCAN; j++) {
            for (size_t i = 0; i < groundScanInd; ++i) {

                lowerInd = j + (i)*Horizon_SCAN;
                upperInd = j + (i + 1) * Horizon_SCAN;

                if (fullCloud->points[lowerInd].intensity == -1 ||
                    fullCloud->points[upperInd].intensity == -1) {
                    // no info to check, invalid 01points
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
                        // upsampling_61(i,j);
                    }
                }
            }
            if (pubClusterCloud.getNumSubscribers() != 0){
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
        double max_x, min_x, max_y, min_y, max_z, min_z;
        //좌우 인덱스를 찾는 부분 한 줄의 행을 모두 저장을 하자

        //좌우 인덱스를 찾는 부분 가장 긴 한 줄의 행렬을 모두 저장을 하자 그런데 여기서 저장을 하면 무슨 의미가 있징....
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

                if (fullCloud->points[thisIndY + thisIndX*Horizon_SCAN].z > 1.5)
                    continue;


                d1 = std::max(rangeMat.at<float>(fromIndX, fromIndY), 
                              rangeMat.at<float>(thisIndX, thisIndY));
                d2 = std::min(rangeMat.at<float>(fromIndX, fromIndY), 
                              rangeMat.at<float>(thisIndX, thisIndY));

                
                if ((*iter).first == 0)
                    alpha = segmentAlphaX;
                else
                    alpha = segmentAlphaY;

                angle = atan2(d2*sin(alpha), (d1 -d2*cos(alpha)));

                if (angle > segmentTheta){

                    queueIndX[queueEndInd] = thisIndX;
                    queueIndY[queueEndInd] = thisIndY;
                    ++queueSize;
                    ++queueEndInd;

                    labelMat.at<int>(thisIndX, thisIndY) = labelCount;
                    lineCountFlag[thisIndX] = true;

                    allPushedIndX[allPushedIndSize] = thisIndX;
                    allPushedIndY[allPushedIndSize] = thisIndY;
                    ++allPushedIndSize;
                }
            }
            
        }

        // check if this segment is valid
        bool feasibleSegment = false;
        if ( 30 <= allPushedIndSize && allPushedIndSize <= 300 )
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
        }
        else{ // segment is invalid, mark these points
            for (size_t i = 0; i < allPushedIndSize; ++i){
                labelMat.at<int>(allPushedIndX[i], allPushedIndY[i]) = 999999;
            }
        }
    }

    // void dbscan(std::vector<std::vector<int>>& clusters, float eps, int min_points)
    // {       
    //     segmentedCloudPure->is_dense=false;
    //     std::vector<int> indices;
    //     pcl::removeNaNFromPointCloud(*segmentedCloudPure, *segmentedCloudPure, indices);
    //     pcl::VoxelGrid<pcl::PointXYZI> vg;
    //     pcl::PointCloud<pcl::PointXYZI>::Ptr downSampledCloud(new pcl::PointCloud<pcl::PointXYZI>);
    //     vg.setInputCloud(segmentedCloudPure);
    //     vg.setLeafSize(0.6f, 0.6f, 0.1f); 
        
    //     vg.filter(*downSampledCloud);
    //     pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);
    //     kdtree->setInputCloud(downSampledCloud);

    //     std::vector<bool> visited(downSampledCloud->points.size(), false);  // 각 포인트가 방문되었는지 여부
    //     std::vector<int> neighbors;                       // 인접 포인트 저장소

    //     for (int i = 0; i < downSampledCloud->points.size(); ++i) {
    //         if (visited[i]) continue;  // 이미 방문한 포인트는 무시

    //         visited[i] = true;
    //         kdtree->radiusSearch(downSampledCloud->points[i], eps, neighbors);  // i번째 포인트와 거리 eps 이내의 포인트를 찾음

    //         // min_points 이상의 인접 포인트가 있을 경우, 새로운 클러스터 생성
    //         if (neighbors.size() >= min_points) {
    //             std::vector<int> cluster;
    //             cluster.push_back(i);

    //             for (size_t j = 0; j < neighbors.size(); ++j) {
    //                 int neighbor_id = neighbors[j];

    //                 if (visited[neighbor_id]) continue;  // 이미 방문한 인접 포인트는 무시

    //                 visited[neighbor_id] = true;
    //                 std::vector<int> neighbor_neighbors;
    //                 kdtree->radiusSearch(downSampledCloud->points[neighbor_id], eps, neighbor_neighbors);

    //                 // i번째 포인트와 합쳐서 하나의 클러스터를 구성할 수 있는 인접 포인트 찾기
    //                 if (neighbor_neighbors.size() >= min_points) {
    //                     neighbors.insert(neighbors.end(), neighbor_neighbors.begin(), neighbor_neighbors.end());
    //                 }

    //                 cluster.push_back(neighbor_id);  // 클러스터에 추가
    //             }

    //             clusters.push_back(cluster);  // 클러스터 추가
    //         }
    //     }
    // }


    void clustering(){
        segmentedCloudPure->is_dense=false;
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*segmentedCloudPure, *segmentedCloudPure, indices);
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        pcl::PointCloud<pcl::PointXYZI>::Ptr downSampledCloud(new pcl::PointCloud<pcl::PointXYZI>);
        vg.setInputCloud(segmentedCloudPure);
        vg.setLeafSize(0.6f, 0.6f, 0.1f); 
        
        vg.filter(*downSampledCloud);
        
        float clusterTolerance = 1.0;
        int minSize = 10;
        int maxSize = 500;
        if (downSampledCloud->points.size() > 0){
            pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
            tree->setInputCloud(downSampledCloud);
            std::vector<pcl::PointIndices> clusterIndices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
            ec.setClusterTolerance(clusterTolerance);
            ec.setMinClusterSize(minSize);
            ec.setMaxClusterSize(maxSize);
            ec.setSearchMethod(tree);
            ec.setInputCloud(downSampledCloud);
            ec.extract(clusterIndices);

            int j = 0;
            for(pcl::PointIndices getIndices: clusterIndices)
            {
                typename pcl::PointCloud<pcl::PointXYZI>::Ptr ClusterCloud (new pcl::PointCloud<pcl::PointXYZI>);

                for(int index : getIndices.indices){ // 여기서 ClusterCloud에 추가시킬 인덱스를 걸러보자 여기에 해당하는 인덱스만 거를거잖아. 
                    pcl::PointXYZI pt = downSampledCloud->points[index];
                    pt.intensity = (float)(j + 1);

                    if(pt.y > -2.0 && pt.y < 2.0){ // 이 부분이 도로의 영역에 해당하면 이 ClusterCloud에 추가를 시켜 주겠다는 내용임
                        ClusterCloud->points.push_back(pt);
                    }
                }
                ClusterCloud->width = ClusterCloud->points.size();
                ClusterCloud->height = 1;
                ClusterCloud->is_dense = true;

                ClusterClouds.push_back(ClusterCloud);
                j += 1;
            } 
        }
    }
    void filteringObject(){
        int Id = 0;     
        for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : ClusterClouds){
            pcl::PointXYZI minPoint, maxPoint;
            pcl::getMinMax3D(*cluster, minPoint, maxPoint);
            ++Id;
            std::queue<double> _theta;
            if( maxPoint.z > 0.3 && (maxPoint.x-minPoint.x > 0.8 && maxPoint.y-minPoint.y>0.5) && (maxPoint.x-minPoint.x<5 && maxPoint.y-minPoint.y<4) && (maxPoint.x-minPoint.x)*(maxPoint.y-minPoint.y)<20 ){
                double ori_x, ori_y, ori_z, ori_w, theta_avg, mean_y, mean_x, theta;
                if (maxPoint.x - minPoint.x > 0.2 && minPoint.x > 0.2){
                    std::tie(theta, mean_x, mean_y) = get_best_theta(*cluster);
                    _theta.push(theta);
                    if (_theta.size() > 10){
                        _theta.pop();
                    }
                    if (abs(theta - _theta.back()) > 20/180*M_PI){
                        theta = _theta.back();
                    }

                    std::cout << mean_y << std::endl;
                    // theta_history.push_back(theta);
                    // if (abs(theta_history.end() - theta) > 30){
                    //     theta = theta_history.end();
                    // }
                    // if (theta_history.size() > 5){
                    //     theta_history.pop();
                    // }
                    // std::tie(ori_x, ori_y, ori_z, ori_w) = euler_to_quaternion(theta);
                    // if (maxPoint.x  < 5.0 &&  (maxPoint.x-minPoint.x < 7.0) &&  (maxPoint.y - minPoint.y < 4.0) ){
                    // double ori_x, ori_y, ori_z, ori_w;
                    // double theta = get_best_theta(*cluster);
                    std::tie(ori_x, ori_y, ori_z, ori_w) = euler_to_quaternion(theta);
                    marker.header.frame_id = "base_link";
                    marker.header.stamp = ros::Time::now();
                    marker.id = Id;
                    marker.type = visualization_msgs::Marker::CUBE;
                    marker.action = visualization_msgs::Marker::ADD;
                    //이 부분 수정 필요 position.x가 이거 기준이 아니라 어떤 edge를 기준으로 가야함
                    marker.pose.position.x = mean_x;//minPoint.x+1.6;
                    marker.pose.position.y = mean_y;//(maxPoint.y + minPoint.y) / 2;
                    marker.pose.position.z = (maxPoint.z + minPoint.z) / 2;

                    marker.pose.orientation.x = ori_x;
                    marker.pose.orientation.y = ori_y;
                    marker.pose.orientation.z = ori_z;
                    marker.pose.orientation.w = ori_w;

                    // marker.scale.x = 3.2; 
                    // marker.scale.y = 1.5; 
                    // marker.scale.z = 2.0; 
                    if (fabs(maxPoint.y-minPoint.y) < 2.0){
                        marker.pose.orientation.x = 0;
                        marker.pose.orientation.y = 0;
                        marker.pose.orientation.z = 0;
                        marker.pose.orientation.w = 0;
                    }

                    marker.scale.x = fabs(maxPoint.x - minPoint.x);
                    marker.scale.y = fabs(maxPoint.y - minPoint.y);
                    marker.scale.z = fabs(maxPoint.z - minPoint.z);

                    marker.color.r = 0.5;
                    marker.color.g = 0.5;
                    marker.color.b = 0.5;
                    marker.color.a = 0.5;

                    marker.lifetime = ros::Duration(0.1);
            
                    markerarray.markers.push_back(marker);
                    markercnt++;
                }
            }
            // else if( (maxPoint.x-minPoint.x < 0.8 && maxPoint.y-minPoint.y>0.8) && (maxPoint.z - minPoint.z < 2.0) && (maxPoint.x-minPoint.x<1.5 && maxPoint.y-minPoint.y<1.5) && (maxPoint.x-minPoint.x)*(maxPoint.y-minPoint.y)<2  ){
            // if (maxPoint.x - minPoint.x < 6.0 && maxPoint.y - minPoint.y < 6.0 && maxPoint.z - minPoint.z < 6.0 && maxPoint.z > 0.2 && minPoint.z < -0.2){
                // double ori_x, ori_y, ori_z, ori_w;
                // if (maxPoint.x - minPoint.x > 0.2 && minPoint.x > 0.2){
                //     marker.header.frame_id = "base_link";
                //     marker.header.stamp = ros::Time::now();
                //     marker.id = Id;
                //     marker.type = visualization_msgs::Marker::CUBE;
                //     marker.action = visualization_msgs::Marker::ADD;
                //     //이 부분 수정 필요 position.x가 이거 기준이 아니라 어떤 edge를 기준으로 가야함
                //     marker.pose.position.x = (maxPoint.x + minPoint.x) / 2;
                //     marker.pose.position.y = (maxPoint.y + minPoint.y) / 2;
                //     marker.pose.position.z = (maxPoint.z + minPoint.z) / 2;

                //     marker.pose.orientation.x = 0;
                //     marker.pose.orientation.y = 0;
                //     marker.pose.orientation.z = 0;
                //     marker.pose.orientation.w = 0;

                //     marker.scale.x = fabs(maxPoint.x - minPoint.x);
                //     marker.scale.y = fabs(maxPoint.y - minPoint.y);
                //     marker.scale.z = fabs(maxPoint.z - minPoint.z);

                //     marker.color.r = 1.0;
                //     marker.color.g = 1.0;
                //     marker.color.b = 1.0;
                //     marker.color.a = 0.7;

                //     marker.lifetime = ros::Duration(0.1);
            
                //     markerarray.markers.push_back(marker);
                //     markercnt++;
                // }
        }    
        
    }



    std::tuple<double, double, double> get_best_theta(const pcl::PointCloud<PointType> &cloud){

        cv::Mat Matrix_pts = cv::Mat::zeros(cloud.points.size(), 2, CV_64FC1);
        double mean_y = 0;
        double mean_x = 0;
        for (size_t  i=0;i<cloud.size();++i){
            Matrix_pts.at<double>(i,0) = cloud.points[i].x;
            Matrix_pts.at<double>(i,1) = cloud.points[i].y;
            if (cloud.points[i].z < 1.3){
                mean_y += cloud.points[i].y;
                mean_x += cloud.points[i].x;
            }

        }
        mean_x = mean_x / cloud.size();
        mean_y = mean_y / cloud.size();
        // std::cout << mean_x << " : " << mean_y << std::endl;
        double dtheta = M_PI/180;
        double minimal_cost = (-1.0)*FLT_MAX;
        double best_theta = FLT_MAX;
        double ori_x, ori_y, ori_z, ori_w;
        cv::Mat e1 = cv::Mat::zeros(1,2,CV_64FC1);
        cv::Mat e2 = cv::Mat::zeros(1,2,CV_64FC1);

        for (size_t k=0;k<89;++k){
            double theta = k*M_PI/180;
            double cost = (-1.0)*FLT_MAX;
            if (theta < 89*M_PI/180){
                e1.at<double>(0,0) = cos(theta);
                e1.at<double>(0,1) = sin(theta);
                e2.at<double>(0,0) = -sin(theta);
                e2.at<double>(0,1) = cos(theta);

                cv::Mat c1 = Matrix_pts*e1.t();
                cv::Mat c2 = Matrix_pts*e2.t();

                cost = variance_criterion(c1, c2);
                if (minimal_cost < cost){
                    minimal_cost = cost;
                    best_theta = theta;
                }
            }
        }
        if (best_theta == FLT_MAX){
            best_theta = -M_PI/2;}
        return std::make_tuple(best_theta, mean_x, mean_y);
    }

    double variance_criterion(const cv::Mat& c1, const cv::Mat& c2){

        std::vector<double> c1_deep;
        std::vector<double> c2_deep;

        for (size_t i = 0; i<c1.rows; ++i){
            for (size_t j =0; j<c1.cols; ++j){
                c1_deep.push_back(c1.at<double>(i,j));
                c2_deep.push_back(c2.at<double>(i,j));
            }
        }
        sort(c1_deep.begin(), c1_deep.end());
        sort(c2_deep.begin(), c2_deep.end());

        int n_c1 = c1_deep.size();
        int n_c2 = c2_deep.size();

        double c1_min = c1_deep[0];
        double c2_min = c2_deep[0];

        double c1_max = c1_deep[n_c1 - 1];
        double c2_max = c2_deep[n_c2 - 1];

        std::vector<double> D1;
        for (double ic1 : c1_deep){
            double dist1 = std::min(sqrt(pow((c1_max - ic1), 2)), sqrt(pow((ic1 - c1_min), 2)));
            D1.push_back(dist1);
        }

        std::vector<double> D2;
        for (double ic2 : c2_deep){
            double dist2 = std::min(sqrt(pow((c2_max - ic2), 2)), sqrt(pow((ic2 - c2_min), 2)));
            D2.push_back(dist2);
        }

        std::vector<double> E1, E2;
        for (size_t i=0; i<D1.size();++i){
            double d1 = D1[i];
            double d2 = D2[i];
            if (d1<d2){
                E1.push_back(d1);
            }
            else{
                E2.push_back(d2);
            }
        }
        //분산 계산하기
        double V1 = 0.0;
        if (!E1.empty()){
            double mean1 = 0.0;
            for (double e1 : E1){
                mean1 += e1;
            }
            mean1 /=E1.size();

            for (double e1 : E1){
                V1+= std::pow(e1-mean1,2);
            }
            V1 = -V1/E1.size();
        }

        double V2 = 0.0;
        if (!E2.empty()){
            double mean2 = 0.0;
            for (double e2 : E2){
                mean2 += e2;
            }
            mean2 /=E2.size();

            for (double e2 : E2){
                V2+= std::pow(e2-mean2,2);
            }
            V2 = -V2/E2.size();
        }
        //더해서 cost 계산
        double gamma = V1+V2;
        return gamma;
    }
    
    void publishCloud(){
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
        }
        pubMarkerArray.publish(markerarray);
        
        
        if (pubSegmentedCloudPure.getNumSubscribers() !=0){
            pcl::toROSMsg(*ClusterCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubClusterCloud.publish(laserCloudTemp);
        }   
        

        // projected full cloud info
        if (pubFullInfoCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*fullInfoCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubFullInfoCloud.publish(laserCloudTemp);
        }
    }
};




int main(int argc, char** argv){

    ros::init(argc, argv, "lego_loam");
    
    ImageProjection IP;
    // ros::NodeHandle nh;
    // message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, "/velodyne_points", 1);
    // message_filters::Subscriber<vision_msgs::Detection2DArray> bbox_sub(nh, "/jaejun/bbox_info", 1);
    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, vision_msgs::Detection2DArray> MySyncPolicy;
    // message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cloud_sub, bbox_sub);

    // sync.registerCallback(boost::bind(&ImageProjection::cloudHandler, &IP, _1, _2));

    std::cout << " come on" << std::endl;
    ROS_INFO("\033[1;32m---->\033[0m Image Projection Started.");

    ros::spin();
    return 0;
}