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
#include <vector>
#include <unordered_set>
#pragma comment(lib , "json/json_vc71_libmtd")
using PointXYZIRT = OusterPointXYZIRT;

#define GRemoval_TH -0.85
#define GRemoval_angle 15.0
class ImageProjection{
private:

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> MySyncPolicy;
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
    ros::Publisher pubMarkerArrayVis;
    ros::Publisher pubRoiCloud;
    ros::Publisher pubTransformedCloud;
    ros::Publisher pubOdometry;
    ros::Publisher pubTransformedRoiCloud;
    ros::Publisher pubParkingSpace;
    ros::Publisher pubConeArray;
    ros::Publisher pubConeCloud;
    ros::Publisher pubEmergencyAlert;
    ros::Publisher pubfovCloud;
    ros::Publisher pubnongroundCloud;

    // Eigen::Affine3f transform = Eigen::Affine3f::Identity();


    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
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
    pcl::PointCloud<PointType>::Ptr fovCloud;
    pcl::PointCloud<PointType>::Ptr nongroundCloud;

    std::vector <pcl::PointCloud<PointType>::Ptr> ClusterClouds;
    std::vector <pcl::PointCloud<PointType>::Ptr> ConeClusterClouds;

    std::vector<float> MapNode_x;
    std::vector<float> MapNode_y;

    PointType nanPoint;
    PointType query;

    cv::Mat rangeMat;
    cv::Mat labelMat;
    cv::Mat groundMat;
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
    visualization_msgs::MarkerArray markerarray_vis;
    visualization_msgs::Marker marker;
    geometry_msgs::PoseStamped local;
    visualization_msgs::Marker parkingspace;
    visualization_msgs::MarkerArray conearray;
    std_msgs::Bool emergency_flag;

    std::vector<std::pair<int8_t, int8_t> > neighborIterator; 
    std::vector<std::pair<float, float>> global_path;
    std::vector<std::pair<double, double> > cones;

    uint16_t *allPushedIndX; 
    uint16_t *allPushedIndY;

    uint16_t *queueIndX; 
    uint16_t *queueIndY;

    uint16_t *RadiusSearchInd;



public:
    ImageProjection():
        nh("~"){

        subLaserClouds.subscribe(nh, "/os_cloud_node/points", 10);
        subLocalMsgs.subscribe(nh, "/local_msgs_to_vision", 1000);
        
        sync.reset(new Sync(MySyncPolicy(50), subLaserClouds,  subLocalMsgs));
        sync->registerCallback(boost::bind(&ImageProjection::cloudHandler, this, _1, _2));

        pubFullCloud = nh.advertise<sensor_msgs::PointCloud2> ("/full_cloud_projected", 1);
        pubFullInfoCloud = nh.advertise<sensor_msgs::PointCloud2> ("/full_cloud_info", 1);
        pubConeCloud = nh.advertise<sensor_msgs::PointCloud2> ("/cone_cloud", 1);

        pubGroundCloud = nh.advertise<sensor_msgs::PointCloud2> ("/ground_cloud", 1);
        pubSegmentedCloud = nh.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud", 1);
        pubSegmentedCloudPure = nh.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud_pure", 1);
        pubOutlierCloud = nh.advertise<sensor_msgs::PointCloud2> ("/outlier_cloud", 1);
        pubMarkerArray = nh.advertise<visualization_msgs::MarkerArray>("/markers", 10);
        pubMarkerArrayVis = nh.advertise<visualization_msgs::MarkerArray>("/markers_vis", 10);
        pubOdometry = nh.advertise<geometry_msgs::PoseStamped>("/local_msg", 10);
        pubTransformedRoiCloud = nh.advertise<sensor_msgs::PointCloud2> ("/transformed_roi_cloud", 1);
        pubParkingSpace = nh.advertise<visualization_msgs::Marker>("/parking_space", 10);
        pubConeArray = nh.advertise<visualization_msgs::MarkerArray>("/cones", 10);
        pubEmergencyAlert = nh.advertise<std_msgs::Bool>("/emergencyAlert", 1);
        pubfovCloud = nh.advertise<sensor_msgs::PointCloud2> ("/fov_cloud", 1);
        pubRoiCloud = nh.advertise<sensor_msgs::PointCloud2> ("/roi_cloud", 1);
        pubTransformedCloud = nh.advertise<sensor_msgs::PointCloud2> ("/transformed_cloud", 1);
        pubnongroundCloud = nh.advertise<sensor_msgs::PointCloud2> ("/nongroundcloud", 1);
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

        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
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
        fovCloud.reset(new pcl::PointCloud<PointType>());
        TransformedCloud.reset(new pcl::PointCloud<PointType>());
        TransformedRoiCloud.reset(new pcl::PointCloud<PointType>());
        interestCloud.reset(new pcl::PointCloud<PointType>());
        coneCloud.reset(new pcl::PointCloud<PointType>());
        nongroundCloud.reset(new pcl::PointCloud<PointType>());

        std::pair<int8_t, int8_t> neighbor;
        neighbor.first = -1; neighbor.second =  0; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second =  1; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second = -1; neighborIterator.push_back(neighbor);
        neighbor.first =  1; neighbor.second =  0; neighborIterator.push_back(neighbor);

        allPushedIndX = new uint16_t[N_SCAN*Horizon_SCAN];
        allPushedIndY = new uint16_t[N_SCAN*Horizon_SCAN];

        queueIndX = new uint16_t[N_SCAN*Horizon_SCAN];
        queueIndY = new uint16_t[N_SCAN*Horizon_SCAN];

        RadiusSearchInd = new uint16_t[N_SCAN*Horizon_SCAN];
    }

    void resetParameters(){
        laserCloudIn->clear();
        groundCloud->clear();
        segmentedCloud->clear();
        segmentedCloudPure->clear();
        outlierCloud->clear();
        coneCloud->clear();
        fovCloud->clear();
        ClusterClouds.clear();
        ClusterCloud->clear();
        ConeClusterClouds.clear();
        ConeClusterCloud->clear();
        nongroundCloud->clear();
        MapNode_x.clear();
        MapNode_y.clear();
        cones.clear();

        markerarray.markers.clear();
        markerarray_vis.markers.clear();

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
        // std::vector<int> indices;
        // pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);
        // std::cout << "====" << std::endl;
        // have "ring" channel in the cloud

    }
    

    
    

    void cloudHandler(const boost::shared_ptr<const sensor_msgs::PointCloud2>& cloudMsg,
    const boost::shared_ptr<const geometry_msgs::PoseStamped>& poseMsg
                    )
        {
        auto startTime = std::chrono::high_resolution_clock::now();
        // 1. Convert ros message to pcl point cloud
        copyPointCloud(cloudMsg);
        projectPointCloud();
        groundRemoval();
        // upsamplePointCloud();
        local_callback(poseMsg);
        transform_to_base();
        clustering();
        // if (curr_index > 100){
        //     clusteringCone();

        // }
        filteringObject();
        filteringCone();
        publishCloud();
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
        t.open("/home/vision/lidar_ws/src/jaejun/maps/obs_small_gwangjang.json");
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
        int step_size = 500;
        int save_idx = 0;
        for (int i = std::max(curr_index - step_size, 0); i < curr_index + step_size; ++i) {
            double dis = std::hypot(global_path[i].first - ego_x, global_path[i].second - ego_y);
            if ((min_dis > dis || min_dis == -1) && save_idx <= i) {
                min_dis = dis;
                min_idx = i;
                save_idx = i;
            }
        }
        curr_index = min_idx;
        std::cout << "current index : " << curr_index << std::endl;
        if (interestCloud->points.size() > 0){
            pcl::KdTreeFLANN<PointType> kdtree;
            kdtree.setInputCloud(interestCloud);
            float radius;
            int path_len = global_path.size();
            int min_lookback = curr_index - std::min(curr_index, 200);
            int max_lookforward = curr_index + std::min(200, path_len - curr_index - 1);
            std::unordered_set<int> all_indices;
            // if (interestCloud->points.size() > 0){
            for(int i = min_lookback; i < max_lookforward;++i){
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
                    std::vector<int> idxes;
                    std::vector<float> sqr_dists;

                    idxes.clear();
                    sqr_dists.clear();
                    // if (curr_index < 1940){
                    //     radius = 2.0;
                    // }
                    // else if (curr_index >= 1940 && curr_index <= 10153){
                    //     radius = 5.5;
                    // }
                    // if (curr_index > 10153){
                    //     radius = 2.3;
                    // }
                    /*
                    평행  10153~
                    소형 1560~1940
                    */
                    radius = 2.0;
                    kdtree.radiusSearch(query, radius, idxes, sqr_dists);
                    
                    for (const auto& idx : idxes) {
                        all_indices.insert(idx);
                        // RoiCloud->points.push_back(interestCloud->points[idx]);
                    }
                }
            } 
            for (const auto& idx : all_indices){
                RoiCloud->points.push_back(interestCloud->points[idx]);
            }
        }
    }

    void projectPointCloud(){
        int emergencyCount = 0;
        float verticalAngle, horizonAngle, range, intensity, roi_x, roi_y, ori, rel_x, rel_y;
        size_t rowIdn, columnIdn, index, cloudSize; 
        PointType thisPoint;
        cloudSize = laserCloudIn->points.size();
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.rotate(Eigen::AngleAxisf(-1.5 * M_PI / 180.0, Eigen::Vector3f::UnitY()));
        transform.translation() << 0.0, 0.0, 0.0;
        pcl::transformPointCloud(*laserCloudIn, *laserCloudIn, transform);

        for (size_t i = 0; i < cloudSize; ++i){

            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            intensity = laserCloudIn->points[i].intensity;

            if (useCloudRing == true){
                rowIdn = 31- laserCloudIn->points[i].ring;
            }
            else{
                verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
                rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
            }
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

            columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
            if (columnIdn >= Horizon_SCAN)
                columnIdn -= Horizon_SCAN;

            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            if (thisPoint.x > 0.3 && thisPoint.x < emergencyRange && thisPoint.y > -0.4 && thisPoint.y < 0.4 && thisPoint.z > -0.3){
                emergencyCount += 1;
                if (emergencyCount >= 10){
                    emergency_flag.data = true;
                }
            }

            range = sqrt((thisPoint.x) * (thisPoint.x) + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
            if (range < 0.6 && thisPoint.x < 0.1){
                continue;
            }

            if ((atan2(thisPoint.x, thisPoint.y)*180/M_PI > 55) && (thisPoint.x < 10) && (atan2(thisPoint.x, thisPoint.y)*180/M_PI < 125)){
            // if (thisPoint.x > 0.1){
                fovCloud->points.push_back(thisPoint);
            }
            
            
            rangeMat.at<float>(rowIdn, columnIdn) = range;

            thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

            index = columnIdn  + rowIdn * Horizon_SCAN;
            thisPoint.x += 1.35;
            fullCloud->points[index] = thisPoint;
            fullInfoCloud->points[index] = thisPoint;
            fullInfoCloud->points[index].intensity = range;


            if (thisPoint.x < 15.0 && thisPoint.x > -2.0 && thisPoint.y < 6.0 && thisPoint.y > -6.0  && thisPoint.z > -0.7 && thisPoint.z < 1.5){// && thisPoint.z < 2.0){
            // if ((atan2(thisPoint.x, thisPoint.y)*180/M_PI > -20) && (atan2(thisPoint.x, thisPoint.y)*180/M_PI < 200)){// && thisPoint.z < 2.0){

                interestCloud->points.push_back(thisPoint);
            }
            
            if (thisPoint.x < 15.0 && thisPoint.x > -1.0 && thisPoint.y < -1.0 && thisPoint.y > -4 && thisPoint.z > -0.55){
                coneCloud->points.push_back(thisPoint);
            }
        }

    }

    void upsamplePointCloud(){
        int index, upper_index, upper_range;
        float x, y, z, z_est, azimuth, upper_rangexy, angle;
        for (int u = 0; u<16; ++u){
            for (int v = 0; v < Horizon_SCAN; ++v){
                if (rangeMat.at<float>(15-u, v) > 2000){
                    upper_range = rangeMat.at<float>(15-u+1, v);
                    index = v + (15-u)*Horizon_SCAN;
                    upper_index = v  + (15-u+1) * Horizon_SCAN;
                    x = fullCloud->points[upper_index].x;
                    y = fullCloud->points[upper_index].y;
                    z = fullCloud->points[upper_index].z;
                    if (z < 0.5){
                        angle = -16.6 + (15-u-1);
                        upper_rangexy = sqrt(x*x + y*y);
                        z_est = upper_rangexy*tan(angle*M_PI/180);
                        rangeMat.at<float>(15-u,v) = sqrt(x*x + y*y + z*z);
                        fullCloud->points[index].x = x;
                        fullCloud->points[index].y = y;
                        fullCloud->points[index].z = z_est;
                        fullCloud->points[index].intensity = 0.5;
                        pcl::PointXYZI points;
                        points.x = x;
                        points.y = y;
                        points.z = z_est;
                        points.intensity = 0.5;
                        if (points.x > 0.0){
                            interestCloud->points.push_back(points);
                        }
                    }
                }
            }
        }
    }

    void groundRemoval(){
        size_t lowerInd, upperInd;
        float diffX, diffY, diffZ, angle, ring;
        // groundMat
        // -1, no valid info to check if ground of not
        //  0, initial value, after validation, means not ground
        //  1, ground
        for (size_t j = 0; j < Horizon_SCAN; ++j){
            for (size_t i = 0; i < groundScanInd; ++i){

                lowerInd = j + ( i )*Horizon_SCAN;
                upperInd = j + (i+1)*Horizon_SCAN;

                diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
                diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
                diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;

                angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;

                if (abs(angle - sensorMountAngle) <= GRemoval_angle && fullCloud->points[lowerInd].z < GRemoval_TH){
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
                    if (groundMat.at<int8_t>(i,j) == 1){
                        groundCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                    }
                    else{
                        nongroundCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                    }
                        
                }
            }
        }
    }

    void cloudSegmentation(){
        for (size_t i = 0; i < N_SCAN; ++i)
            for (size_t j = 0; j < Horizon_SCAN; ++j)
                if (labelMat.at<int>(i,j) == 0)
                    labelComponents(i, j);

        int sizeOfSegCloud = 0;
        for (size_t i = 0; i < N_SCAN; ++i) {
            for (size_t j = 0; j < Horizon_SCAN; ++j) {
                if (labelMat.at<int>(i,j) > 0 || groundMat.at<int8_t>(i,j) == 1){
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
        
        for (size_t i = 0; i < N_SCAN; ++i){
            for (size_t j = 0; j < Horizon_SCAN; ++j){
                if (labelMat.at<int>(i,j) > 0 && labelMat.at<int>(i,j) != 999999){
                    segmentedCloudPure->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                    segmentedCloudPure->points.back().intensity = labelMat.at<int>(i,j);
                }
            }
        }
    }

    void labelComponents(int row, int col){
        float d1, d2, alpha, angle, dist;
        int fromIndX, fromIndY, thisIndX, thisIndY; 
        bool lineCountFlag[N_SCAN] = {false};
        bool heightFlag[N_SCAN] = {false};
        float maxHeight = 0.0;
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
                
                maxHeight = std::max(fullCloud->points[thisIndY + thisIndX*Horizon_SCAN].z, maxHeight);

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
                if (dist < 0.4){                
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
        
        bool feasibleSegment = false;
        if ( 5 <= allPushedIndSize && allPushedIndSize <= 1000)
            feasibleSegment = true;
        else if (allPushedIndSize >= segmentValidPointNum  && allPushedIndSize <= 1000){
            int lineCount = 0;
            for (size_t i = 0; i < N_SCAN; ++i)
                if (lineCountFlag[i] == true)
                    ++lineCount;
            if (lineCount >= segmentValidLineNum)
                feasibleSegment = true;
        }
        
        if (feasibleSegment == true){
            ++labelCount;
        }
        else{ 
            for (size_t i = 0; i < allPushedIndSize; ++i){
                labelMat.at<int>(allPushedIndX[i], allPushedIndY[i]) = 999999;
            }
        }
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
    }

    void filteringCone(){
        int Id = 0;
        for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : ClusterClouds){
            pcl::PointXYZI minPoint, maxPoint;
            pcl::getMinMax3D(*cluster, minPoint, maxPoint);
            ++Id;
            std::queue<double> _theta;
            double ori_x, ori_y, ori_z, ori_w,r, theta_avg, mean_y, mean_x,mid_x, mid_y, thetas, theta, pos_x, pos_y;
            // if(maxPoint.z < 0.0 && maxPoint.y < -0.8 && minPoint.y > -4.0 && minPoint.x > -1.0 && maxPoint.x < 15.0 && (maxPoint.x-minPoint.x) < 0.6 && (maxPoint.y-minPoint.y) < 0.6){
            if(maxPoint.z < 0.5 && maxPoint.y < -0.8 && minPoint.y > -4.0 && minPoint.x > -1.0 && maxPoint.x < 15.0){

                visualization_msgs::Marker conemarker;
                conemarker.header.frame_id = "base_laser";
                conemarker.header.stamp = ros::Time::now();
                conemarker.id = Id;
                conemarker.type = visualization_msgs::Marker::CUBE;
                conemarker.action = visualization_msgs::Marker::ADD;
                    
                conemarker.pose.position.x = (maxPoint.x + minPoint.x)/2;
                conemarker.pose.position.y = (maxPoint.y + minPoint.y)/2;
                conemarker.pose.position.z = -0.5; 
                std::pair<float, float> cone;
                cone.first = (maxPoint.x + minPoint.x)/2, cone.second = (maxPoint.y + minPoint.y)/2;
                cones.push_back(cone);

                conemarker.pose.orientation.x = 0.0;
                conemarker.pose.orientation.y = 0.0;
                conemarker.pose.orientation.z = 0.0;
                conemarker.pose.orientation.w = 1.0;

                conemarker.scale.x = fabs(maxPoint.x - minPoint.x);
                conemarker.scale.y = fabs(maxPoint.y - minPoint.y);
                conemarker.scale.z = 0.6;

                conemarker.color.r = 0.0;
                conemarker.color.g = 0.0;
                conemarker.color.b = 1.0;
                conemarker.color.a = 1.0;

                conemarker.lifetime = ros::Duration(0.1);

                conearray.markers.push_back(conemarker);
                
            }
        }     

        std::sort(cones.begin(), cones.end(), compare);

        if (cones.size() >= 2)    {    
            for (int i = 0; i < cones.size()-1; i++){
            float distance = std::sqrt(std::pow(cones[i].first - cones[i+1].first, 2)
                + std::pow(cones[i].second - cones[i+1].second, 2));
                if (distance >= 3.8 && distance <= 5.25 ) {
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
                }
            }
        }
    }

    
    void clustering(){
        auto startTime = std::chrono::high_resolution_clock::now();
        
        RoiCloud->is_dense=false;
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*RoiCloud, *RoiCloud, indices);
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr floorRemoved(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PassThrough<pcl::PointXYZI> pass;
        pass.setInputCloud(RoiCloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-0.8, 1.5);
        pass.filter(*floorRemoved);
        
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        pcl::PointCloud<pcl::PointXYZI>::Ptr downSampledCloud(new pcl::PointCloud<pcl::PointXYZI>);
        vg.setInputCloud(floorRemoved);
        vg.setLeafSize(0.1f, 0.1f, 0.1f);
        vg.filter(*downSampledCloud);



        float clusterTolerance = 1.0;
        int minSize = 3;
        int maxSize = 3000;
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

                for(int index : getIndices.indices){ 
                    pcl::PointXYZI pt = downSampledCloud->points[index];
                    pt.intensity = (float)(j + 1);

                    ClusterCloud->points.push_back(pt);
                }

                ClusterCloud->width = ClusterCloud->points.size();
                ClusterCloud->height = 1;
                ClusterCloud->is_dense = true;

                ClusterClouds.push_back(ClusterCloud);
                j += 1;
            } 
        }
        

        std::cout << "number of clusters  : " << ClusterClouds.size() << std::endl;
    }

    void filteringObject(){
        int Id = 0;   
        for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : ClusterClouds){
            pcl::PointXYZI minPoint, maxPoint;
            pcl::getMinMax3D(*cluster, minPoint, maxPoint);
            ++Id;
            double ori_x, ori_y, ori_z, ori_w,r, theta_avg, mean_y, mean_x,mid_x, mid_y, thetas, theta, pos_x, pos_y;
            if ((maxPoint.x - minPoint.x) < 7.0  && (maxPoint.y - minPoint.y) < 6.0){
                std::tie(theta, mean_x, mean_y) = get_best_theta(*cluster);
                visualization_msgs::Marker bbox;
                bbox = bbox_3d(*cluster, heading, ego_x, ego_y, Id);
                marker.header.frame_id = "base_link";
                marker.header.stamp = ros::Time::now();
                marker.type = visualization_msgs::Marker::CUBE;
                marker.action = visualization_msgs::Marker::ADD;

                pos_x = mean_x*cos(-heading*M_PI/180) + mean_y*sin(-heading*M_PI/180);
                pos_y = -mean_x*sin(-heading*M_PI/180) + mean_y*cos(-heading*M_PI/180); 
                // if (sqrt(mean_x*mean_x + mean_y*mean_y) < 6.7){

                marker.pose.position.x = pos_x + ego_x;
                marker.pose.position.y = pos_y + ego_y;
                marker.pose.position.z = (maxPoint.z + minPoint.z) / 2;

                marker.pose.orientation.x = 0;
                marker.pose.orientation.y = 0;
                marker.pose.orientation.z = 0;
                marker.pose.orientation.w = 0;

                marker.scale.x = fabs(maxPoint.x - minPoint.x);
                marker.scale.y = fabs(maxPoint.y - minPoint.y);
                marker.scale.z = fabs(maxPoint.z - minPoint.z);

                marker.color.r = 0.8;
                marker.color.g = 0.8;
                marker.color.b = 0.8;
                marker.color.a = 1.0;
                marker.id = Id;
                marker.lifetime = ros::Duration(0.1);

                markerarray.markers.push_back(marker);
                markerarray_vis.markers.push_back(bbox);
                // }
                markercnt ++;
            }
        }      
    }

   


    void publishCloud(){

        sensor_msgs::PointCloud2 laserCloudTemp;
        pubMarkerArray.publish(markerarray);
        pubMarkerArrayVis.publish(markerarray_vis);

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

        pcl::toROSMsg(*segmentedCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "base_laser";
        pubSegmentedCloud.publish(laserCloudTemp);
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
        if (pubGroundCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*groundCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_laser";
            pubGroundCloud.publish(laserCloudTemp);
        }
        if (pubSegmentedCloudPure.getNumSubscribers() != 0){
            pcl::toROSMsg(*segmentedCloudPure, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_laser";
            pubSegmentedCloudPure.publish(laserCloudTemp);
        }
        if (pubFullInfoCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*fullInfoCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_laser";
            pubFullInfoCloud.publish(laserCloudTemp);
        }
        if (pubnongroundCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*nongroundCloud, laserCloudTemp);
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
        if (pubfovCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*fovCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubfovCloud.publish(laserCloudTemp);
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