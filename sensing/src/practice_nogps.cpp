#include "utility.h"
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32MultiArray.h>
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
#include <pcl/filters/passthrough.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <math.h>
#include <chrono>
#include <tuple>
#include <queue>
#include <fstream>
#include <utility>
#include <json/json.h>
#pragma comment(lib , "json/json_vc71_libmtd")

#define GRemoval_TH -0.7
#define GRemoval_angle 15.0
class ImageProjection{
private:

    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, sensor_msgs::PointCloud2> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;

    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::PointCloud2> subLaserClouds;
    message_filters::Subscriber<geometry_msgs::PoseStamped> subLocalMsgs;
    boost::shared_ptr<Sync> sync;

    //timesynchronizer
    // message_filters::Subscriber<sensor_msgs::PointCloud2> subLaserClouds;
    // message_filters::Subscriber<geometry_msgs::PoseStamped> subLocalMsgs;

    // typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, sensor_msgs::PointCloud2> MySyncPolicy;
    // typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    // boost::shared_ptr<Sync> sync;


    ros::Subscriber subLaserCloud;
    ros::Subscriber subNode;
    
    ros::Publisher pubFullCloud;
    ros::Publisher pubFullInfoCloud;

    ros::Publisher pubGroundCloud;
    ros::Publisher pubSegmentedCloud;
    ros::Publisher pubSegmentedCloudPure;
    ros::Publisher pubOutlierCloud;
    ros::Publisher pubMarkerArray;
    ros::Publisher pubRoiCloud;
    ros::Publisher pubTransformedCloud;
    ros::Publisher pubOdometry;
    ros::Publisher pubTransformedRoiCloud;
    ros::Publisher pubParkingSpace;
    ros::Publisher pubConeArray;
    ros::Publisher pubConeCloud;
    ros::Publisher pubEmergencyAlert;

    // Eigen::Affine3f transform = Eigen::Affine3f::Identity();


    pcl::PointCloud<PointType>::Ptr laserCloudIn;
    pcl::PointCloud<PointXYZIR>::Ptr laserCloudInRing;

    pcl::PointCloud<PointType>::Ptr fullCloud; // projected velodyne raw cloud, but saved in the form of 1-D matrix
    pcl::PointCloud<PointType>::Ptr fullInfoCloud; // same as fullCloud, but with intensity - range

    pcl::PointCloud<PointType>::Ptr groundCloud;
    pcl::PointCloud<PointType>::Ptr segmentedCloud;
    pcl::PointCloud<PointType>::Ptr segmentedCloudPure;
    pcl::PointCloud<PointType>::Ptr outlierCloud;
    pcl::PointCloud<PointType>::Ptr ClusterCloud;
    pcl::PointCloud<PointType>::Ptr RoiCloud;
    pcl::PointCloud<PointType>::Ptr TransformedCloud;
    pcl::PointCloud<PointType>::Ptr interestCloud;
    pcl::PointCloud<PointType>::Ptr coneCloud;
    pcl::PointCloud<PointType>::Ptr ConeClusterCloud;
    pcl::PointCloud<PointType>::Ptr TransformedRoiCloud;

    std::vector <pcl::PointCloud<PointType>::Ptr> ClusterClouds;
    std::vector <pcl::PointCloud<PointType>::Ptr> ConeClusterClouds;

    std::vector<float> MapNode_x;
    std::vector<float> MapNode_y;

    PointType nanPoint; // fill in fullCloud at each iteration
    PointType query;

    cv::Mat rangeMat; // range matrix for range image
    cv::Mat labelMat; // label matrix for segmentaiton marking
    cv::Mat groundMat; // ground matrix for ground cloud marking
    int labelCount;
    int markercnt = 0;
    int conecount = 0;

    float startOrientation;
    float endOrientation;

    double ego_x;
    double ego_y;
    double heading;
    double speed_from_gps;
    double speed_from_encoder;
    int curr_index;

    std_msgs::Header cloudHeader;
    visualization_msgs::MarkerArray markerarray;
    visualization_msgs::Marker marker;
    geometry_msgs::PoseStamped local;
    visualization_msgs::Marker parkingspace;
    visualization_msgs::MarkerArray conearray;
    std_msgs::Bool emergency_flag;

    // geometry_msgs::PoseArray posearray;
    // geometry_msgs::Pose pose;


    std::vector<std::pair<int8_t, int8_t> > neighborIterator; // neighbor iterator for segmentaiton process
    std::vector<std::pair<float, float>> global_path;
    std::vector<std::pair<double, double> > cones;

    uint16_t *allPushedIndX; // array for tracking points of a segmented object
    uint16_t *allPushedIndY;

    uint16_t *queueIndX; // array for breadth-first search process of segmentation, for speed
    uint16_t *queueIndY;

    



public:
    ImageProjection():
        nh("~"){

        // subNode = nh.subscribe<std_msgs::Float32MultiArray>("/further_node", 1, &ImageProjection::crop, this);
        // subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/ouster/points", 1, &ImageProjection::cloudHandler, this);

        // subLaserClouds.subscribe(nh, "/os_cloud_node/points", 1);
        // subLocalMsgs.subscribe(nh, "/local_msg", 1);
        // sync.reset(new Sync(MySyncPolicy(10), subLaserClouds, subLocalMsgs));
        // sync->registerCallback(boost::bind(&ImageProjection::cloudHandler,this, _1, _2));
        subLaserClouds.subscribe(nh, "/os_cloud_node/points", 10);
        subLocalMsgs.subscribe(nh, "/local_msgs_to_vision", 10);

        sync.reset(new Sync(MySyncPolicy(100), subLocalMsgs, subLaserClouds));
        sync->registerCallback(boost::bind(&ImageProjection::cloudHandler, this, _1, _2));

        pubFullCloud = nh.advertise<sensor_msgs::PointCloud2> ("/full_cloud_projected", 1);
        pubFullInfoCloud = nh.advertise<sensor_msgs::PointCloud2> ("/full_cloud_info", 1);
        pubConeCloud = nh.advertise<sensor_msgs::PointCloud2> ("/cone_cloud", 1);

        pubGroundCloud = nh.advertise<sensor_msgs::PointCloud2> ("/ground_cloud", 1);
        pubSegmentedCloud = nh.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud", 1);
        pubSegmentedCloudPure = nh.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud_pure", 1);
        pubOutlierCloud = nh.advertise<sensor_msgs::PointCloud2> ("/outlier_cloud", 1);
        pubMarkerArray = nh.advertise<visualization_msgs::MarkerArray>("/markers", 10);
        pubOdometry = nh.advertise<geometry_msgs::PoseStamped>("/local_msg", 10);
        pubTransformedRoiCloud = nh.advertise<sensor_msgs::PointCloud2> ("/transformed_roi_cloud", 1);
        pubParkingSpace = nh.advertise<visualization_msgs::Marker>("/parking_space", 10);
        pubConeArray = nh.advertise<visualization_msgs::MarkerArray>("/cones", 10);
        pubEmergencyAlert = nh.advertise<std_msgs::Bool>("/emergencyAlert", 1);

        pubRoiCloud = nh.advertise<sensor_msgs::PointCloud2> ("/roi_cloud", 1);
        pubTransformedCloud = nh.advertise<sensor_msgs::PointCloud2> ("/transformed_cloud", 1);

        nanPoint.x = std::numeric_limits<float>::quiet_NaN();
        nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        nanPoint.z = std::numeric_limits<float>::quiet_NaN();
        nanPoint.intensity = -1;

        query.x = 0.0f;
        query.y = 0.0f;
        query.z = 0.0f;
        query.intensity = 0.0f;

        map_reader();
        allocateMemory();
        resetParameters();      

    }

    void allocateMemory(){

        laserCloudIn.reset(new pcl::PointCloud<PointType>());
        laserCloudInRing.reset(new pcl::PointCloud<PointXYZIR>());

        fullCloud.reset(new pcl::PointCloud<PointType>());
        fullInfoCloud.reset(new pcl::PointCloud<PointType>());

        groundCloud.reset(new pcl::PointCloud<PointType>());
        segmentedCloud.reset(new pcl::PointCloud<PointType>());
        segmentedCloudPure.reset(new pcl::PointCloud<PointType>());
        outlierCloud.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(N_SCAN*Horizon_SCAN);
        fullInfoCloud->points.resize(N_SCAN*Horizon_SCAN);

        ClusterCloud.reset(new pcl::PointCloud<PointType>());
        ConeClusterCloud.reset(new pcl::PointCloud<PointType>());

        RoiCloud.reset(new pcl::PointCloud<PointType>());
        TransformedCloud.reset(new pcl::PointCloud<PointType>());
        TransformedRoiCloud.reset(new pcl::PointCloud<PointType>());
        interestCloud.reset(new pcl::PointCloud<PointType>());
        coneCloud.reset(new pcl::PointCloud<PointType>());

        std::pair<int8_t, int8_t> neighbor;
        neighbor.first = -1; neighbor.second =  0; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second =  1; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second = -1; neighborIterator.push_back(neighbor);
        neighbor.first =  1; neighbor.second =  0; neighborIterator.push_back(neighbor);

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
        outlierCloud->clear();
        ClusterCloud->clear();
        coneCloud->clear();
        ClusterClouds.clear();
        ConeClusterClouds.clear();
        MapNode_x.clear();
        MapNode_y.clear();
        cones.clear();

        markerarray.markers.clear();
        conearray.markers.clear();
        parkingspace.points.clear();
        // posearray.poses.clear();

        RoiCloud->clear();
        TransformedCloud->clear();
        TransformedRoiCloud->clear();
        interestCloud->clear();

        emergency_flag.data = false;

        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
        groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
        labelMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));
        labelCount = 1;

        std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
        std::fill(fullInfoCloud->points.begin(), fullInfoCloud->points.end(), nanPoint);
        markercnt = 0;


    }

    ~ImageProjection(){}


    void copyPointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

        cloudHeader = laserCloudMsg->header;
        cloudHeader.stamp = ros::Time::now(); // Ouster lidar users may need to uncomment this line
        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
        // Remove Nan points
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);
        // have "ring" channel in the cloud

    }
    

    
    

    void cloudHandler(const boost::shared_ptr<const geometry_msgs::PoseStamped>& poseMsg,
                    const boost::shared_ptr<const sensor_msgs::PointCloud2>& cloudMsg)
        {
        auto startTime = std::chrono::high_resolution_clock::now();
        // 1. Convert ros message to pcl point cloud
        copyPointCloud(cloudMsg);
        // Road_Roi();
        // 2. Start and end angle of a scan
        // 3. Range image projection
        projectPointCloud();
        local_callback(poseMsg);
        // 4. Mark ground points
        groundRemoval();
        // 5. Point cloud segmentation
        // cloudSegmentation();

        transform_to_base();
        // if (conecount < 100){
        clustering();
        // filteringCone();
        // }
        filteringObject();

        // 6. Publish all clouds
        publishCloud();
        // 7. Reset parameters for next iteration
        resetParameters();
        auto endTime = std::chrono::high_resolution_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
        double elapsedSeconds = elapsedTime.count() / 1000000.0;

        std::cout << "Elapsed time: " << elapsedSeconds << std::endl;
    }


    void map_reader() {
        Json::Value root;		
        Json::Reader reader;
        std::ifstream t;
        string index;
        t.open("/home/vision/lidar_ws/src/jaejun/maps/spring_reverse_demo.json");
        // t.open("/home/vision/lidar_ws/src/jaejun/maps/SLAM_imu_offset.json");
        if (!reader.parse(t, root)) {
            cout << "Parsing Failed" << endl;
        }
        for (int k=0;k<root.size();++k){
            index = to_string(k);
            double x = root[index][0].asDouble();
            double y = root[index][1].asDouble();
            global_path.emplace_back(x, y);
        }
        std::cout << "Global Path Created" << std::endl;
        curr_index = 0;
    }

    void local_callback(const geometry_msgs::PoseStampedConstPtr& poseMsg){
        pcl::KdTreeFLANN<PointType> kdtree;
        kdtree.setInputCloud(interestCloud);
        ego_x = poseMsg->pose.position.x;
        ego_y = poseMsg->pose.position.y;
        heading = poseMsg->pose.orientation.w;
        speed_from_gps = poseMsg->pose.orientation.x;
        speed_from_encoder = poseMsg->pose.orientation.y;

        local.pose.position.x = ego_x;
        local.pose.position.y = ego_y;
        local.pose.orientation.w = heading;

        double relative_node_x, relative_node_y;
        double heading_rad = heading * DEG_TO_RAD;

        double min_dis = -1;
        int min_idx = 0;
        int step_size = 50;
        int save_idx = 0;
        for (int i = std::max(curr_index - step_size, 0); i < curr_index + step_size; ++i) {
            // std::cout << global_path[i].first << std::endl;
            
            double dis = std::hypot(global_path[i].first - ego_x, global_path[i].second - ego_y);
            if ((min_dis > dis || min_dis == -1) && save_idx <= i) {
                min_dis = dis;
                min_idx = i;
                save_idx = i;
            }
            
        }
        curr_index = min_idx;
        int path_len = global_path.size();
        int min_lookback = std::min(curr_index, 200);
        int max_lookforward = std::min(200, path_len - curr_index - 1);
        // for(int i = -min_lookback; i < max_lookforward;++i){
        std::cout << curr_index << std::endl;
        for(int i = curr_index - std::min(200, curr_index); i < curr_index + std::min(200, 2051-curr_index);++i){
            if (i%10==0){
                double dx = global_path[i].first - ego_x;
                double dy = global_path[i].second - ego_y;

                double cos_heading = std::cos(heading_rad);
                double sin_heading = std::sin(heading_rad);

                relative_node_x = cos_heading * dx + sin_heading * dy;
                relative_node_y = -sin_heading * dx + cos_heading * dy;
                
                query.x = static_cast<float>(relative_node_x);
                query.y = static_cast<float>(relative_node_y);
                query.z = 0.0f;
                query.intensity = 0.0f;
                // std::cout << query.x << ", " << query.y << std::endl;

                std::vector<int> idxes;
                std::vector<float> sqr_dists;

                idxes.clear();
                sqr_dists.clear();
                
                float radius = 1.6;

                kdtree.radiusSearch(query, radius, idxes, sqr_dists);
                for (const auto& idx : idxes) {
                    RoiCloud->points.push_back(interestCloud->points[idx]);
                }

            }
        }   
        // std::cout << curr_index << std::endl;
    }

    void projectPointCloud(){
        // range image projection
        int emergencyCount = 0;
        float verticalAngle, horizonAngle, range, intensity, roi_x, roi_y, ori, rel_x, rel_y;
        size_t rowIdn, columnIdn, index, cloudSize; 
        PointType thisPoint;
        cloudSize = laserCloudIn->points.size();

        for (size_t i = 0; i < cloudSize; ++i){
            
            int left = 0;
            int right = MapNode_x.size() - 1;
            int loc = 0;
            int mid = (left+right)/2;


            thisPoint.x = laserCloudIn->points[i].x + 1.36;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            intensity = laserCloudIn->points[i].intensity;

            // find the row and column index in the iamge for this point
            if (useCloudRing == true){
                rowIdn = laserCloudInRing->points[i].ring;
            }
            else{
                verticalAngle = atan2(thisPoint.z, sqrt((thisPoint.x - 1.36)* (thisPoint.x-1.36) + thisPoint.y * thisPoint.y)) * 180 / M_PI;
                rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
            }
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            horizonAngle = atan2(thisPoint.x - 1.36, thisPoint.y) * 180 / M_PI;

            columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
            if (columnIdn >= Horizon_SCAN)
                columnIdn -= Horizon_SCAN;

            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            if (thisPoint.x - 1.36 > 0.3 && thisPoint.x - 1.36 < emergencyRange && thisPoint.y > -0.4 && thisPoint.y < 0.4 && thisPoint.z > -0.3){
                emergencyCount += 1;
                if (emergencyCount >= 10 && curr_index < 854){
                    emergency_flag.data = true;
                }

            }
            range = sqrt((thisPoint.x - 1.36) * (thisPoint.x - 1.36) + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
            if (range < sensorMinimumRange)
                continue;
            
            rangeMat.at<float>(rowIdn, columnIdn) = range;

            thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

            index = columnIdn  + rowIdn * Horizon_SCAN;
            fullCloud->points[index] = thisPoint;
            fullInfoCloud->points[index] = thisPoint;
            fullInfoCloud->points[index].intensity = range; // the corresponding range of a point is saved as "intensity"
            
            if (thisPoint.x < 15.0 && thisPoint.x > -1.0 && thisPoint.y > -8.0 && thisPoint.y < 8.0){//} && thisPoint.z > -0.6 && thisPoint.z < 2.0){
                interestCloud->points.push_back(thisPoint);
            }

            if (thisPoint.x < 15.0 && thisPoint.x > -1.0 && thisPoint.y < -1.0 && thisPoint.y > -3.8 && thisPoint.z > -0.4){
                coneCloud->points.push_back(thisPoint);
            }
            //roi
            // if (right > 0){
            //     if (thisPoint.x < MapNode_x.back() && thisPoint.x > -0.1){
            //         while(left <=right){
            //             mid = (left+right)/2;          
            //             if (thisPoint.x == MapNode_x[mid]){
            //                 break;
            //             }
            //             else if (thisPoint.x < MapNode_x[mid]){
            //                 right = mid-1;
            //             }
            //             else{
            //                 left = mid+1;
            //             }
            //         }
            //         if (abs(MapNode_y[left] - thisPoint.y) < laneWidth){
            //             std::cout << "lane calculating" << std::endl;
            //             RoiCloud->points.push_back(thisPoint);

            //         }
            //     }
            // }
        }
    }
    

    void groundRemoval(){
        size_t lowerInd, upperInd;
        float diffX, diffY, diffZ, angle;
        // groundMat
        // -1, no valid info to check if ground of not
        //  0, initial value, after validation, means not ground
        //  1, ground
        for (size_t j = 0; j < Horizon_SCAN; ++j){
            for (size_t i = 0; i < groundScanInd; ++i){

                lowerInd = j + ( i )*Horizon_SCAN;
                upperInd = j + (i+1)*Horizon_SCAN;

                // if (fullCloud->points[lowerInd].z < -0.6)
                //     groundMat.at<int8_t>(i,j) = 1;
                //     continue;

                if (fullCloud->points[lowerInd].intensity == -1 ||
                    fullCloud->points[upperInd].intensity == -1){
                    // no info to check, invalid points
                    groundMat.at<int8_t>(i,j) = -1;
                    continue;
                }
                    
                diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
                diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
                diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;

                angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;

                // if (abs(angle - sensorMountAngle) <= 10 || fullCloud->points[lowerInd].z < 0.65){
                if (fullCloud->points[lowerInd].z < -0.65){

                    groundMat.at<int8_t>(i,j) = 1;
                    groundMat.at<int8_t>(i+1,j) = 1;
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
        for (size_t i = 0; i < N_SCAN; ++i){
            for (size_t j = 0; j < Horizon_SCAN; ++j){
                if (groundMat.at<int8_t>(i,j) == 1 || rangeMat.at<float>(i,j) == FLT_MAX){
                    labelMat.at<int>(i,j) = -1;
                }
            }
        }
        if (pubGroundCloud.getNumSubscribers() != 0){
            for (size_t i = 0; i <= groundScanInd; ++i){
                for (size_t j = 0; j < Horizon_SCAN; ++j){
                    if (groundMat.at<int8_t>(i,j) == 1)
                        groundCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
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

                    // save seg cloud
                    segmentedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                    // size of seg cloud
                    ++sizeOfSegCloud;
                }
            }

        }
        
        // extract segmented cloud for visualization
        if (pubSegmentedCloudPure.getNumSubscribers() != 0){
            for (size_t i = 0; i < N_SCAN; ++i){
                for (size_t j = 0; j < Horizon_SCAN; ++j){
                    if (labelMat.at<int>(i,j) > 0 && labelMat.at<int>(i,j) != 999999){
                        // fullCloud->points[j + i*Horizon_SCAN].intensity = labelMat.at<int>(i, j);
                        segmentedCloudPure->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                        segmentedCloudPure->points.back().intensity = labelMat.at<int>(i,j);
                        if (segmentedCloudPure->points[j+i*Horizon_SCAN].x < 0 && segmentedCloudPure->points[j+i*Horizon_SCAN].x > -1 && segmentedCloudPure->points[j+i*Horizon_SCAN].y > 0 && segmentedCloudPure->points[j+i*Horizon_SCAN].y < 2)
                            std::cout << labelMat.at<int>(i, j) << std::endl;
                    }
                }
            }
        }
    }

    void labelComponents(int row, int col){
        // use std::queue std::vector std::deque will slow the program down greatly
        float d1, d2, alpha, angle, dist;
        int fromIndX, fromIndY, thisIndX, thisIndY; 
        bool lineCountFlag[N_SCAN] = {false};
        bool heightFlag[N_SCAN] = {false};

        queueIndX[0] = row;
        queueIndY[0] = col;
        int queueSize = 1;
        int queueStartInd = 0;
        int queueEndInd = 1;

        allPushedIndX[0] = row;
        allPushedIndY[0] = col;
        int allPushedIndSize = 1;
        
        while(queueSize > 0){

            fromIndX = queueIndX[queueStartInd];
            fromIndY = queueIndY[queueStartInd];
            --queueSize;
            ++queueStartInd;

            labelMat.at<int>(fromIndX, fromIndY) = labelCount;

            for (auto iter = neighborIterator.begin(); iter != neighborIterator.end(); ++iter){

                thisIndX = fromIndX + (*iter).first;
                thisIndY = fromIndY + (*iter).second;
                if (thisIndX < 0 || thisIndX >= N_SCAN)
                    continue;
                if (thisIndY < 0)
                    thisIndY = Horizon_SCAN - 1;
                if (thisIndY >= Horizon_SCAN)
                    thisIndY = 0;

                if (labelMat.at<int>(thisIndX, thisIndY) != 0)
                    continue;
                
                if (fullCloud->points[thisIndY + thisIndX*Horizon_SCAN].z > 1.3)
                    continue;    

                d1 = std::max(rangeMat.at<float>(fromIndX, fromIndY), 
                              rangeMat.at<float>(thisIndX, thisIndY));
                d2 = std::min(rangeMat.at<float>(fromIndX, fromIndY), 
                              rangeMat.at<float>(thisIndX, thisIndY));

                if ((*iter).first == 0)
                    alpha = segmentAlphaX;
                else
                    alpha = segmentAlphaY;

                angle = atan2(d2*sin(alpha), (d1 - d2*cos(alpha)));

                dist = (d1 - d2*cos(alpha))/cos(angle) ;
                if (d1 < 20 && angle > segmentTheta && dist < 0.4){// && fullCloud->points[thisIndY + thisIndX*Horizon_SCAN].z > -0.6){
                // if (fullCloud->points[thisIndY + thisIndX*Horizon_SCAN].z > -0.7){
                
                    queueIndX[queueEndInd] = thisIndX;
                    queueIndY[queueEndInd] = thisIndY;
                    ++queueSize;
                    ++queueEndInd;

                    labelMat.at<int>(thisIndX, thisIndY) = labelCount;

                    lineCountFlag[thisIndX] = true;
                    if (fullCloud->points[thisIndY + thisIndX*Horizon_SCAN].z > 2.0)
                        heightFlag[thisIndX] = true;

                    allPushedIndX[allPushedIndSize] = thisIndX;
                    allPushedIndY[allPushedIndSize] = thisIndY;
                    ++allPushedIndSize;
                }
            }
        }
        bool feasibleSegment = false;
        if ( 30 <= allPushedIndSize)
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
        // height ê³ ë ¤
        // check if this segment is valid
        // bool feasibleSegment = false;
        // if (allPushedIndSize >= 30){
        //     feasibleSegment = true;
        //     for (size_t j=0; j < N_SCAN; ++j){
        //         if (heightFlag[j] == true)
        //             feasibleSegment = false;
        //         }
        // }

        // else if (allPushedIndSize >= segmentValidPointNum){
        //     int lineCount = 0;
        //     for (size_t i = 0; i < N_SCAN; ++i){
        //         if (heightFlag[i] == true){
        //             feasibleSegment = false;
        //             break;
        //             }
        //         if (lineCountFlag[i] == true)
        //             ++lineCount;
        //     }
        //     if (lineCount >= segmentValidLineNum)
        //         feasibleSegment = true;            
        // }
        // // segment is valid, mark these points
        // if (feasibleSegment == true){
        //     ++labelCount;
        // }else{ // segment is invalid, mark these points
        //     for (size_t i = 0; i < allPushedIndSize; ++i){
        //         labelMat.at<int>(allPushedIndX[i], allPushedIndY[i]) = 999999;
        //     }.
        // }

    }

    void transform_to_base(){
        PointType thisPoint;
        double laneWidth = 4.0;
        float intensity;
        int left = 0;
        int right = MapNode_x.size() - 1;
        int loc = 0;
        int mid = (left+right)/2;
        size_t cloudSize; 

        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.rotate(Eigen::AngleAxisf(heading * M_PI / 180.0, Eigen::Vector3f::UnitZ()));
        transform.translation() << ego_x, ego_y, 0.0;
        pcl::transformPointCloud(*fullCloud, *TransformedCloud, transform);
        pcl::transformPointCloud(*RoiCloud, *TransformedRoiCloud, transform);

        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr floor_removed(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PassThrough<pcl::PointXYZI> pass;

        // // pass.setInputCloud(TransformedCloud);
        // // pass.setFilterFieldName("y");
        // // pass.setFilterLimits(-6.0, 6.0);
        // // pass.filter(*filtered_cloud);
        pass.setInputCloud(TransformedCloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-0.45, 1.1);
        pass.filter(*floor_removed);

        // for (size_t i = 0; i < floor_removed->points.size(); ++i){
            
        //     int left = 0;
        //     int right = MapNode_x.size() - 1;
        //     int loc = 0.0;
        //     int mid = (left+right)/2;


        //     thisPoint.x = floor_removed->points[i].x;
        //     thisPoint.y = floor_removed->points[i].y;
        //     thisPoint.z = floor_removed->points[i].z;
        //     intensity = floor_removed->points[i].intensity;

            // if (right > 0){
            //     if (thisPoint.y < *std::max_element(MapNode_y.begin(), MapNode_y.end()));{//MapNode_x.back()){
            //         while(left <=right){
            //             mid = (left+right)/2;          
            //             if (thisPoint.y == MapNode_y[mid]){
            //                 break;
            //             }
            //             else if (thisPoint.y < MapNode_y[mid]){
            //                 right = mid-1;
            //             }
            //             else{
            //                 left = mid+1;
            //             }
            //         }
            //         if (std::hypot(MapNode_x[left] - thisPoint.x ,MapNode_y[left] - thisPoint.y) < laneWidth){
            //         // if (fabs(MapNode_y[left] - thisPoint.y) < laneWidth){
            //             RoiCloud->points.push_back(thisPoint);
            //         }
            //     }
            // }
        // }    
    }


    void clustering(){
        coneCloud->is_dense=false;
        std::cout << "clusteringggggggg" << std::endl;
        std::vector<int> indices;
        // pcl::removeNaNFromPointCloud(*segmentedCloudPure, *segmentedCloudPure, indices);
        pcl::removeNaNFromPointCloud(*coneCloud, *coneCloud, indices);
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr floorRemoved(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PassThrough<pcl::PointXYZI> pass;
        pass.setInputCloud(coneCloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-0.5, 0.5);
        pass.filter(*floorRemoved);
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        pcl::PointCloud<pcl::PointXYZI>::Ptr downSampledCloud(new pcl::PointCloud<pcl::PointXYZI>);
        vg.setInputCloud(floorRemoved);
        vg.setLeafSize(0.2f, 0.2f, 0.1f); 
        vg.filter(*downSampledCloud);



        float clusterTolerance = 0.7;
        int minSize = 3;
        int maxSize = 1000;
        if (coneCloud->points.size() > 0){
            pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
            tree->setInputCloud(coneCloud);
            std::vector<pcl::PointIndices> clusterIndices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
            ec.setClusterTolerance(clusterTolerance);
            ec.setMinClusterSize(minSize);
            ec.setMaxClusterSize(maxSize);
            ec.setSearchMethod(tree);
            ec.setInputCloud(coneCloud);
            ec.extract(clusterIndices);

            int j = 0;
            for(pcl::PointIndices getIndices: clusterIndices)
            {
                typename pcl::PointCloud<pcl::PointXYZI>::Ptr ClusterCloud (new pcl::PointCloud<pcl::PointXYZI>);

                for(int index : getIndices.indices){ 
                    pcl::PointXYZI pt = coneCloud->points[index];
                    pt.intensity = (float)(j + 1);

                    // if(pt.y > -10.0 && pt.y < 10.0){ 
                    ClusterCloud->points.push_back(pt);
                    // }
                }

                ClusterCloud->width = ClusterCloud->points.size();
                ClusterCloud->height = 1;
                ClusterCloud->is_dense = true;

                ClusterClouds.push_back(ClusterCloud);
                j += 1;
            } 
        }
        

        // float coneEps = 0.7;
        // int coneMin = 10;
        // int coneMax = 1000;

        // if (coneCloud->points.size() > 0){
        //     pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        //     tree->setInputCloud(coneCloud);
        //     std::vector<pcl::PointIndices> clusterIndices;
        //     pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        //     ec.setClusterTolerance(coneEps);
        //     ec.setMinClusterSize(coneMin);
        //     ec.setMaxClusterSize(coneMax);
        //     ec.setSearchMethod(tree);
        //     ec.setInputCloud(coneCloud);
        //     ec.extract(clusterIndices);

        //     int j = 0;
        //     for(pcl::PointIndices getIndices: clusterIndices)
        //     {
        //         typename pcl::PointCloud<pcl::PointXYZI>::Ptr ClusterCloud (new pcl::PointCloud<pcl::PointXYZI>);

        //         for(int index : getIndices.indices){ 
        //             pcl::PointXYZI pt = coneCloud->points[index];
        //             pt.intensity = (float)(j + 1);

        //             // if(pt.y > -10.0 && pt.y < 10.0){ 
        //             ConeClusterCloud->points.push_back(pt);
        //             // }
        //         }

        //         ConeClusterCloud->width = ConeClusterCloud->points.size();
        //         ConeClusterCloud->height = 1;
        //         ConeClusterCloud->is_dense = true;

        //         ConeClusterClouds.push_back(ConeClusterCloud);
        //         j += 1;
        //     } 
        // }
    }
    
    void filteringObject(){
        int Id = 0;   

        for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : ClusterClouds){
            pcl::PointXYZI minPoint, maxPoint;
            pcl::getMinMax3D(*cluster, minPoint, maxPoint);
            ++Id;
            std::queue<double> _theta;

            // if( minPoint.z < 0.1 && maxPoint.z < 1.5 && (maxPoint.x-minPoint.x > 0.05 && maxPoint.y-minPoint.y>0.05) && (maxPoint.x-minPoint.x<5 && maxPoint.y-minPoint.y<4) && (maxPoint.x-minPoint.x)*(maxPoint.y-minPoint.y)<20 ){
            // double ori_x, ori_y, ori_z, ori_w,r, theta_avg, mean_y, mean_x,mid_x, mid_y, thetas, theta, pos_x, pos_y;
            // if ((maxPoint.x - minPoint.x > 0.2) && (maxPoint.x - minPoint.x < 6.0) && (maxPoint.y - minPoint.y > 0.2) && (maxPoint.y - minPoint.y < 6.0)){
            //     std::tie(theta, mean_x, mean_y) = get_best_theta(*cluster);
            //     _theta.push(theta);
            //     if (_theta.size() > 10){
            //         _theta.pop();
            //     }
            //     if (abs(theta - _theta.back()) > 20/180*M_PI){
            //         theta = _theta.back();
            //     }

            //     // std::cout << mean_y << std::endl;
            //     // theta_history.push_back(theta);
            //     // if (abs(theta_history.end() - theta) > 30){
            //     //     theta = theta_history.end();
            //     // }
            //     // if (theta_history.size() > 5){
            //     //     theta_history.pop();
            //     // }
            //     // std::tie(ori_x, ori_y, ori_z, ori_w) = euler_to_quaternion(theta);
            //     // if (maxPoint.x  < 5.0 &&  (maxPoint.x-minPoint.x < 7.0) &&  (maxPoint.y - minPoint.y < 4.0) ){
            //     // double ori_x, ori_y, ori_z, ori_w;
            //     // double theta = get_best_theta(*cluster);
            //     std::tie(ori_x, ori_y, ori_z, ori_w) = euler_to_quaternion(-heading*M_PI/180);
            //     marker.header.frame_id = "base_link";
            //     marker.header.stamp = ros::Time::now();
            //     marker.id = Id;
            //     marker.type = visualization_msgs::Marker::CUBE;
            //     marker.action = visualization_msgs::Marker::ADD;
            //     //이 부분 수정 필요 position.x가 이거 기준이 아니라 어떤 edge를 기준으로 가야함
                
            //     // pos_x = mean_x*cos(90-heading) + mean_y*sin(heading-90);
            //     // pos_y = -mean_x*sin(90-heading) + mean_y*cos(heading-90);
            //     mid_x = (maxPoint.x + minPoint.x) / 2;
            //     mid_y = (maxPoint.y + minPoint.y) / 2;
            //     pos_x = mean_x*cos(-heading*M_PI/180) + mean_y*sin(-heading*M_PI/180);
            //     pos_y = -mean_x*sin(-heading*M_PI/180) + mean_y*cos(-heading*M_PI/180); 



            //     // thetas = heading*M_PI/180 + atan2(mid_x, mid_y);
            //     // std::cout << atan2(mid_x, mid_y) << std::endl;
            //     // r = sqrt(mid_x*mid_x + mid_y*mid_y);
            //     // pos_x = r*cos(thetas);
            //     // pos_y = r*sin(thetas);

            //     marker.pose.position.x = pos_x + ego_x;//(maxPoint.y + minPoint.y) / 2;//mean_x;//minPoint.x+1.6;
            //     marker.pose.position.y = pos_y + ego_y;//(maxPoint.x + minPoint.x) / 2;//mean_y;//(maxPoint.y + minPoint.y) / 2;
            //     marker.pose.position.z = 0; //(maxPoint.z + minPoint.z) / 2;
            //     // marker.pose.orientation.x = ori_x;
            //     // marker.pose.orientation.y = ori_y;
            //     // marker.pose.orientation.z = ori_z;
            //     // marker.pose.orientation.w = ori_w;

            //     // marker.scale.x = 3.2; 
            //     // marker.scale.y = 1.5; 
            //     // marker.scale.z = 2.0; 
            //     // if (fabs(maxPoint.y-minPoint.y) < 2.0){
            //     marker.pose.orientation.x = 0;
            //     marker.pose.orientation.y = 0;
            //     marker.pose.orientation.z = 0;
            //     marker.pose.orientation.w = 0;
            //     // }

            //     marker.scale.x = fabs(maxPoint.x - minPoint.x);
            //     marker.scale.y = fabs(maxPoint.y - minPoint.y);
            //     marker.scale.z = fabs(maxPoint.z - minPoint.z);

            //     marker.color.r = 0.8;
            //     marker.color.g = 0.8;
            //     marker.color.b = 0.8;
            //     marker.color.a = 1.0;

            //     marker.lifetime = ros::Duration(0.1);
        
            //     markerarray.markers.push_back(marker);
            // //     markercnt ++;
            // }

            if(maxPoint.z < 0.5 && maxPoint.y < -0.8 && minPoint.y > -4.0 && minPoint.x > -1.0 && maxPoint.x < 15.0 && (maxPoint.x-minPoint.x) < 0.6 && (maxPoint.y-minPoint.y) < 0.6){
                visualization_msgs::Marker conemarker;
                std::cout << "Ci bong bang gu" << std::endl;
                conemarker.header.frame_id = "base_link";
                conemarker.header.stamp = ros::Time::now();
                conemarker.id = Id;
                conemarker.type = visualization_msgs::Marker::CUBE;
                conemarker.action = visualization_msgs::Marker::ADD;
                // if((maxPoint.y + minPoint.y) / 2 < -0.5){
                    
                conemarker.pose.position.x = (maxPoint.x + minPoint.x)/2;//minPoint.x+1.6;
                conemarker.pose.position.y = (maxPoint.y + minPoint.y)/2;//(maxPoint.y + minPoint.y) / 2;
                conemarker.pose.position.z = -0.5; //(maxPoint.z + minPoint.z) / 2;
                std::cout << maxPoint.x << " : " << minPoint.x << std::endl;
                std::pair<float, float> cone;
                cone.first = (maxPoint.x + minPoint.x)/2, cone.second = (maxPoint.y + minPoint.y)/2;
                cones.push_back(cone);

                conemarker.pose.orientation.x = 0.0;
                conemarker.pose.orientation.y = 0.0;
                conemarker.pose.orientation.z = 0.0;
                conemarker.pose.orientation.w = 1.0;

                conemarker.scale.x = fabs(maxPoint.x - minPoint.x);
                conemarker.scale.y = fabs(maxPoint.y - minPoint.y);
                conemarker.scale.z = 0.6;//fabs(maxPoint.z - minPoint.z);

                conemarker.color.r = 0.0;
                conemarker.color.g = 0.0;
                conemarker.color.b = 1.0;
                conemarker.color.a = 1.0;

                conemarker.lifetime = ros::Duration(0.1);
        
                conearray.markers.push_back(conemarker);
                
            }
        }     
        std::sort(cones.begin(), cones.end(), compare);


        // 여기가 문제임 씨봉방거
        if (cones.size() >= 2)    {    
            std::cout << "hhhhhhhhhhhhhhhhhhhhh" << std::endl;
            for (int i = 0; i < cones.size()-1; i++){
            float distance = std::sqrt(std::pow(cones[i].first - cones[i+1].first, 2)
                + std::pow(cones[i].second - cones[i+1].second, 2));
                if (distance >= 3.5 && distance <= 5.2 ) {
                    std::cout << "===========" << std::endl;
                    double x1, y1, pos_x1, pos_y1, x2, y2, pos_x2, pos_y2;
                    parkingspace.id = 0;
                    parkingspace.type = visualization_msgs::Marker::LINE_STRIP;
                    parkingspace.action = visualization_msgs::Marker::ADD;
                    parkingspace.scale.x = 0.1;
                    parkingspace.color.r = 0.0;
                    parkingspace.color.g = 1.0;
                    parkingspace.color.b = 0.0;
                    parkingspace.color.a = 1.0;
                    geometry_msgs::Point point1, point2;

                    x1 = cones[i].first;
                    y1 = cones[i].second;
                    std::cout << "x1 is " << x1 << std::endl;
                    pos_x1 = x1*cos(-heading*M_PI/180) + y1*sin(-heading*M_PI/180) + ego_x;
                    pos_y1 = -x1*sin(-heading*M_PI/180) + y1*cos(-heading*M_PI/180) + ego_y; 
                    point1.x = pos_x1;
                    point1.y = pos_y1;
                    point1.z = 0.0;
                    
                    x2 = cones[i+1].first;
                    y2 = cones[i+1].second;
                    pos_x2 = x2*cos(-heading*M_PI/180) + y2*sin(-heading*M_PI/180) + ego_x;
                    pos_y2 = -x2*sin(-heading*M_PI/180) + y2*cos(-heading*M_PI/180) + ego_y; 
                    point2.x = pos_x2;
                    point2.y = pos_y2;
                    point2.z = 0.0;

                    parkingspace.points.push_back(point1);
                    parkingspace.points.push_back(point2);
                    conecount += 1;
                    std::cout << "conecount is : " << conecount << std::endl;
                }
            }
        }
    }        

    // void filteringCone(){
    //     int Id = 0;     
    //     double mean_y, mean_x, theta, pos_x, pos_y;
    //     std::cout << "================" << std::endl;
    //     for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : ClusterCloud){
    //         if (max){
    //             std::tie(theta, mean_x, mean_y) = get_best_theta(*cluster);
    //             pcl::PointXYZI minPoint, maxPoint;
    //             pcl::getMinMax3D(*cluster, minPoint, maxPoint);
    //             ++Id;
    //             std::queue<double> _theta;
    //             visualization_msgs::Marker conemarker;
    //             // if( (maxPoint.z < 0.0) && (maxPoint.y < -0.8 && minPoint.y > -4.0) && (minPoint.x > -1.0 && maxPoint.x < 20.0) && (maxPoint.x-minPoint.x < 0.6 && maxPoint.y-minPoint.y < 0.6)){
                    
    //             conemarker.header.frame_id = "base_link";
    //             conemarker.header.stamp = ros::Time::now();
    //             conemarker.id = Id;
    //             conemarker.type = visualization_msgs::Marker::CUBE;
    //             conemarker.action = visualization_msgs::Marker::ADD;
    //             // if((maxPoint.y + minPoint.y) / 2 < -0.5){
                    
    //             conemarker.pose.position.x = mean_x;//minPoint.x+1.6;
    //             conemarker.pose.position.y = mean_y;//(maxPoint.y + minPoint.y) / 2;
    //             conemarker.pose.position.z = -0.5; //(maxPoint.z + minPoint.z) / 2;
                
    //             std::pair<float, float> cone;
    //             cone.first = mean_x, cone.second = mean_y;
    //             cones.push_back(cone);

    //             conemarker.pose.orientation.x = 0.0;
    //             conemarker.pose.orientation.y = 0.0;
    //             conemarker.pose.orientation.z = 0.0;
    //             conemarker.pose.orientation.w = 1.0;

    //             conemarker.scale.x = 0.4;//fabs(maxPoint.x - minPoint.x);
    //             conemarker.scale.y = 0.4;//fabs(maxPoint.y - minPoint.y);
    //             conemarker.scale.z = 0.6;//fabs(maxPoint.z - minPoint.z);

    //             conemarker.color.r = 0.0;
    //             conemarker.color.g = 0.0;
    //             conemarker.color.b = 1.0;
    //             conemarker.color.a = 1.0;

    //             conemarker.lifetime = ros::Duration(0.1);
        
    //             conearray.markers.push_back(conemarker);
    //             markercnt ++;
    //             // }
                    
    //         }  
    //     }

    //     // pair sort x기준
    //     std::sort(cones.begin(), cones.end(), compare);
    //     for (int i = 0; i < cones.size()-1; i++){
    //         float distance = std::sqrt(std::pow(cones[i].first - cones[i+1].first, 2)
    //             + std::pow(cones[i].second - cones[i+1].second, 2));
    //         if (distance >= 3.0 && distance <= 5.5 && (fabs(cones[i].second - cones[i+1].second < 1.0))) {
    //             std::cout << "===========" << std::endl;
    //             double x1, y1, pos_x1, pos_y1, x2, y2, pos_x2, pos_y2;
    //             parkingspace.id = 0;
    //             parkingspace.type = visualization_msgs::Marker::LINE_STRIP;
    //             parkingspace.action = visualization_msgs::Marker::ADD;
    //             parkingspace.scale.x = 0.1;
    //             parkingspace.color.r = 0.0;
    //             parkingspace.color.g = 1.0;
    //             parkingspace.color.b = 0.0;
    //             parkingspace.color.a = 1.0;
    //             geometry_msgs::Point point1, point2;

    //             x1 = cones[i].first;
    //             y1 = cones[i].second;
    //             pos_x1 = x1*cos(-heading*M_PI/180) + y1*sin(-heading*M_PI/180) + ego_x;
    //             pos_y1 = -x1*sin(-heading*M_PI/180) + y1*cos(-heading*M_PI/180) + ego_y; 
    //             point1.x = pos_x1;
    //             point1.y = pos_y1;
    //             point1.z = 0.0;
                
    //             x2 = cones[i+1].first;
    //             y2 = cones[i+1].second;
    //             pos_x2 = x2*cos(-heading*M_PI/180) + y2*sin(-heading*M_PI/180) + ego_x;
    //             pos_y2 = -x2*sin(-heading*M_PI/180) + y2*cos(-heading*M_PI/180) + ego_y; 
    //             point2.x = pos_x2;
    //             point2.y = pos_y2;
    //             point2.z = 0.0;

    //             parkingspace.points.push_back(point1);
    //             parkingspace.points.push_back(point2);
    //             conecount += 1;
    //             std::cout << "conecount is : " << conecount << std::endl;
    //         }
    //     }

    //     // for (int i = 0; i < conearray.markers.size() - 1; ++i) {
    //     //     for (int j = i + 1; j < conearray.markers.size(); ++j) {
    //     //         float distance = std::sqrt(std::pow(conearray.markers[i].pose.position.x - conearray.markers[j].pose.position.x, 2)
    //     //                                 + std::pow(conearray.markers[i].pose.position.y - conearray.markers[j].pose.position.y, 2));
    //     //         if (distance >= 3.0 && distance <= 5.5 && (fabs(conearray.markers[i].pose.position.y - conearray.markers[j].pose.position.y < 1.0))) {
    //     //             std::cout << "===========" << std::endl;
    //     //             double x1, y1, pos_x1, pos_y1, x2, y2, pos_x2, pos_y2;
    //     //             parkingspace.id = 0;
    //     //             parkingspace.type = visualization_msgs::Marker::LINE_STRIP;
    //     //             parkingspace.action = visualization_msgs::Marker::ADD;
    //     //             parkingspace.scale.x = 0.1;
    //     //             parkingspace.color.r = 0.0;
    //     //             parkingspace.color.g = 1.0;
    //     //             parkingspace.color.b = 0.0;
    //     //             parkingspace.color.a = 1.0;
    //     //             geometry_msgs::Point point1, point2;

    //     //             x1 = conearray.markers[i].pose.position.x;
    //     //             y1 = conearray.markers[i].pose.position.y;
    //     //             pos_x1 = x1*cos(-heading*M_PI/180) + y1*sin(-heading*M_PI/180) + ego_x;
    //     //             pos_y1 = -x1*sin(-heading*M_PI/180) + y1*cos(-heading*M_PI/180) + ego_y; 
    //     //             point1.x = pos_x1;
    //     //             point1.y = pos_y1;
    //     //             point1.z = 0.0;
                    
    //     //             x2 = conearray.markers[j].pose.position.x;
    //     //             y2 = conearray.markers[j].pose.position.y;
    //     //             pos_x2 = x2*cos(-heading*M_PI/180) + y2*sin(-heading*M_PI/180) + ego_x;
    //     //             pos_y2 = -x2*sin(-heading*M_PI/180) + y2*cos(-heading*M_PI/180) + ego_y; 
    //     //             point2.x = pos_x2;
    //     //             point2.y = pos_y2;
    //     //             point2.z = 0.0;

    //     //             parkingspace.points.push_back(point1);
    //     //             parkingspace.points.push_back(point2);
    //     //             conecount += 1;
    //     //             std::cout << "conecount is : " << conecount << std::endl;
    //     //         }
    //     //     }
    //     // }
    
    // }      



    void publishCloud(){
        // 1. Publish Seg Cloud Info

        // 2. Publish clouds
        sensor_msgs::PointCloud2 laserCloudTemp;

        // markerarray.header.stamp = cloudHeader.stamp;
        // markerarray.header.frame_id = "base_link";
        pubMarkerArray.publish(markerarray);
        pubEmergencyAlert.publish(emergency_flag);

        pubConeArray.publish(conearray);
        local.header.stamp = cloudHeader.stamp;
        local.header.frame_id = "base_laser";
        pubOdometry.publish(local);

        parkingspace.header.frame_id = "base_link";
        parkingspace.header.stamp = ros::Time::now();
        pubParkingSpace.publish(parkingspace);

        pcl::toROSMsg(*outlierCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "base_laser";
        pubOutlierCloud.publish(laserCloudTemp);
        // segmented cloud with ground
        pcl::toROSMsg(*segmentedCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "base_laser";
        pubSegmentedCloud.publish(laserCloudTemp);
        // projected full cloud
        if (pubFullCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*fullCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_laser";
            pubFullCloud.publish(laserCloudTemp);
        }
        if (pubConeCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*coneCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubConeCloud.publish(laserCloudTemp);
        }
        // original dense ground cloud
        if (pubGroundCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*groundCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_laser";
            pubGroundCloud.publish(laserCloudTemp);
        }
        // segmented cloud without ground
        if (pubSegmentedCloudPure.getNumSubscribers() != 0){
            pcl::toROSMsg(*segmentedCloudPure, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_laser";
            pubSegmentedCloudPure.publish(laserCloudTemp);
        }
        // projected full cloud info
        if (pubFullInfoCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*fullInfoCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_laser";
            pubFullInfoCloud.publish(laserCloudTemp);
        }
        if (pubRoiCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*RoiCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_laser";
            pubRoiCloud.publish(laserCloudTemp);
        }
        if (pubTransformedCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*TransformedCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubTransformedCloud.publish(laserCloudTemp);
        }
        if (pubTransformedRoiCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*TransformedRoiCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubTransformedRoiCloud.publish(laserCloudTemp);
        }
    }
};




int main(int argc, char** argv){

    ros::init(argc, argv, "lego_loam");
    
    ImageProjection IP;

    ROS_INFO("\033[1;32m---->\033[0m Image Projection Started.");

    ros::spin();
    return 0;
}