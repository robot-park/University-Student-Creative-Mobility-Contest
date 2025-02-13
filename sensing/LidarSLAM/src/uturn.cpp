#include "parameters.h"

typedef message_filters::sync_policies::ApproximateTime< sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> MySyncPolicy;
typedef message_filters::Synchronizer<MySyncPolicy> Sync;

class U_turn{
private:

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::PointCloud2> subLaserCloud;
    message_filters::Subscriber<geometry_msgs::PoseStamped> subLocalMsg;
    boost::shared_ptr<Sync> sync;
    //ros::Subscriber subLaserCloud;

    ros::Publisher pubCone_coordinate;
    ros::Publisher pubBarrel_coordinate;

    ros::Publisher pubConeROICloud;
    ros::Publisher pubBarrelROICloud;

    ros::Publisher pubCurrentEgo;
    ros::Publisher pubUturnSign;

    visualization_msgs::MarkerArray ego_markerarray;
    visualization_msgs::MarkerArray cone_markerarray;
    visualization_msgs::MarkerArray barrel_markerarray;
    visualization_msgs::MarkerArray uturn;

    vector<PointType> cone_coordinate;
    vector<PointType> barrel_coordinate;

    pcl::PointCloud<PointType>::Ptr laserCloudIn;//ouster lidar 센서값 받아오는 용도

    pcl::PointCloud<PointType>::Ptr coneROICloud;//콘의 point들 저장용 
    pcl::PointCloud<PointType>::Ptr barrelROICloud;//콘의 point들 저장용 
    pcl::PointCloud<PointType>::Ptr coneCloud;//콘의 point들 저장용 
    pcl::PointCloud<PointType>::Ptr barrelCloud;//콘의 point들 저장용 

    tf::StampedTransform MappedTrans;
    tf::TransformBroadcaster tfBroadcaster;

    tf::StampedTransform map_2_camera_init_Trans;
    tf::TransformBroadcaster tfBroadcasterMap2CameraInit;

    tf::StampedTransform camera_2_base_link_Trans;
    tf::TransformBroadcaster tfBroadcasterCamera2Baselink;
    
    double timeLaserOdometry;
    
    float transformTobeMapped[6];

    bool set_Uturn_position = false;
    std_msgs::Header cloudHeader;
    
public:
    U_turn():
        nh("~"){
 
        subLaserCloud.subscribe(nh, "/uturn_laser", 10);
        subLocalMsg.subscribe(nh, "/local_msgs_to_vision", 1000);

        sync.reset(new Sync(MySyncPolicy(100), subLaserCloud, subLocalMsg));
        sync->registerCallback(boost::bind(&U_turn::cloudHandler, this, _1, _2));
        
        //subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/os_cloud_node/points", 1, &U_turn::cloudHandler, this);
        
        pubConeROICloud= nh.advertise<sensor_msgs::PointCloud2> ("/coneROICloud", 1);
        pubBarrelROICloud= nh.advertise<sensor_msgs::PointCloud2> ("/barrelROICloud", 1);

        pubCurrentEgo = nh.advertise<visualization_msgs::MarkerArray>("/currnet_ego", 10);

        pubUturnSign = nh.advertise<visualization_msgs::MarkerArray>("/Uturn", 10);

        pubCone_coordinate = nh.advertise<visualization_msgs::MarkerArray>("/cones", 10);
        pubBarrel_coordinate = nh.advertise<visualization_msgs::MarkerArray>("/barrels", 10);

        MappedTrans.frame_id_ = "/camera_init";
        MappedTrans.child_frame_id_ = "/camera";

        map_2_camera_init_Trans.frame_id_ = "/map";
        map_2_camera_init_Trans.child_frame_id_ = "/camera_init";

        camera_2_base_link_Trans.frame_id_ = "/camera";
        camera_2_base_link_Trans.child_frame_id_ = "/base_link";

        allocateMemory();
        resetParameters();
    }

    void allocateMemory(){

        laserCloudIn.reset(new pcl::PointCloud<PointType>());

        coneROICloud.reset(new pcl::PointCloud<PointType>());
        barrelROICloud.reset(new pcl::PointCloud<PointType>());
        coneCloud.reset(new pcl::PointCloud<PointType>());
        barrelCloud.reset(new pcl::PointCloud<PointType>());
        
        ego_markerarray.markers.clear();

        cone_markerarray.markers.clear();
        barrel_markerarray.markers.clear();
        uturn.markers.clear();

        cone_coordinate.clear();
        barrel_coordinate.clear();

        set_Uturn_position = false;

        for (int i = 0; i < 6; ++i){
            transformTobeMapped[i] = 0;
        }
    }

    void resetParameters(){

        laserCloudIn->clear();
        
        coneROICloud->clear();
        barrelROICloud->clear();
        

        ego_markerarray.markers.clear();
        uturn.markers.clear();
        cone_markerarray.markers.clear();
        barrel_markerarray.markers.clear();

        //cone_coordinate.clear();
        // barrel_coordinate.clear();

    }

    ~U_turn(){}
    void LocalMsgHandler(const geometry_msgs::PoseStamped::ConstPtr& LocalMsg){

        timeLaserOdometry = LocalMsg->header.stamp.toSec();

        transformTobeMapped[0] = 0.0;
        transformTobeMapped[1] = 0.0;
        transformTobeMapped[2] = LocalMsg->pose.orientation.w*PI/180;

        transformTobeMapped[3] = LocalMsg->pose.position.x;
        transformTobeMapped[4] = LocalMsg->pose.position.y;
        transformTobeMapped[5] = 0.0;
        
    }

    void cloudHandler(const boost::shared_ptr<const sensor_msgs::PointCloud2>& laserCloudMsg,
                    const boost::shared_ptr<const geometry_msgs::PoseStamped>& LocalMsg){
                        
        copyPointCloud(laserCloudMsg);

        LocalMsgHandler(LocalMsg);

        filter_cones_cloud();// 설정한 roi로 점들을 filtering

        find_cone(coneROICloud);//카메라 퓨전을 활용하여 콘에 해당하는 point들만 남김

        //if(!set_Uturn_position)
        push_back_barrel(barrelROICloud);

        publishCloud();
        publishTF();
        resetParameters();
    }

    PointTypePose trans2PointTypePose(float transformIn[]){
        PointTypePose thisPose6D;
        thisPose6D.x = transformIn[3];
        thisPose6D.y = transformIn[4];
        thisPose6D.z = transformIn[5];
        thisPose6D.roll  = transformIn[0];
        thisPose6D.pitch = transformIn[1];
        thisPose6D.yaw   = transformIn[2];
        return thisPose6D;
    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn){

        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        PointType *pointFrom;
        PointType pointTo;

        int cloudSize = cloudIn->points.size();
        cloudOut->resize(cloudSize);
        
        for (int i = 0; i < cloudSize; ++i){

            pointFrom = &cloudIn->points[i];
            float x1 = cos(transformIn->yaw) * pointFrom->x - sin(transformIn->yaw) * pointFrom->y;
            float y1 = sin(transformIn->yaw) * pointFrom->x + cos(transformIn->yaw)* pointFrom->y;
            float z1 = pointFrom->z;

            float x2 = x1;
            float y2 = cos(transformIn->roll) * y1 - sin(transformIn->roll) * z1;
            float z2 = sin(transformIn->roll) * y1 + cos(transformIn->roll)* z1;

            pointTo.x = cos(transformIn->pitch) * x2 + sin(transformIn->pitch) * z2 + transformIn->x;
            pointTo.y = y2 + transformIn->y;
            pointTo.z = -sin(transformIn->pitch) * x2 + cos(transformIn->pitch) * z2 + transformIn->z;
            pointTo.intensity = pointFrom->intensity;

            cloudOut->points[i] = pointTo;
        }
        return cloudOut;
    }

    void copyPointCloud(const sensor_msgs::PointCloud2::ConstPtr& laserCloudMsg){

        cloudHeader = laserCloudMsg->header;
        cloudHeader.stamp = ros::Time::now(); // Ouster lidar users may need to uncomment this line
        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
        // Remove Nan points
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);
    }

    void filter_cones_cloud(){

        size_t cloudSize; 
        PointType thisPoint;

        cloudSize = laserCloudIn->points.size();

        for (size_t i = 0; i < cloudSize; ++i){

            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            
            if(set_ConeROi(thisPoint) ){
                thisPoint.x = thisPoint.x + lidar_to_gps_D;

                coneROICloud->push_back(thisPoint);
            }
            else if(set_BarrelROi(thisPoint)){
                thisPoint.x = thisPoint.x + lidar_to_gps_D;
                
                barrelROICloud->push_back(thisPoint);
            }
        }
    }

    bool set_ConeROi(const PointType thisPoint){//roi 설정 
        double radius = w_length-erp_side;

        if(thisPoint.z < 0.1 && thisPoint.z > -lidar_height + 0.20 && thisPoint.x >0 && thisPoint.y > -erp_side && thisPoint.y < w_length){ 
            return true;
        }
        else return false;
    }

    bool set_BarrelROi(const PointType thisPoint){//roi 설정 
        double radius = w_length-erp_side;

        if(thisPoint.z < 0.1 && thisPoint.z > -lidar_height + 0.20 && thisPoint.x >0.5 && thisPoint.x < 3 && thisPoint.y > -8.0 && thisPoint.y < -erp_side){ 
            return true;
        }
        else return false;
    }

    void find_cone(pcl::PointCloud<PointType>::Ptr conecloud){
        if(conecloud->points.size() !=0){
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *coneCloud += *transformPointCloud(conecloud,&thisPose6D); 
            clustering(0,coneCloud);
            PointType ego;
            ego.x =thisPose6D.x;
            ego.y =thisPose6D.y;
            ego.z =thisPose6D.z;

            if(cone_coordinate.size() >= 2){
                PointType closest_cone =  find_closest_barrel(ego,cone_coordinate);

                float distanceSquared = calculate_distance_squared(ego,closest_cone);

                if(distanceSquared <= uturn_point*uturn_point){
                    set_Uturn_position = true;
                    cout <<"uturn!!!!" << endl;
                }
                cout << "Detected Cone: " << cone_coordinate.size() <<endl;
            }
        }
    }

    void push_back_barrel(pcl::PointCloud<PointType>::Ptr barrellocalcloud){
        if(barrellocalcloud->points.size() !=0){
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *barrelCloud += *transformPointCloud(barrellocalcloud,&thisPose6D); 
            clustering(1,barrelCloud);
        }
    }

    void clustering(int mode, pcl::PointCloud<PointType>::Ptr conecloud){
        
        std::vector<int> indice;
        pcl::removeNaNFromPointCloud(*conecloud,*conecloud,indice);

        map<double, PointType> cone_center_point;

        PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
        
        pcl::VoxelGrid<PointType> sor;
        sor.setInputCloud(conecloud);  // 입력 클라우드 설정
        sor.setLeafSize(0.1f, 0.1f, 0.1f);  // Voxel 크기 설정 (x, y, z)

        pcl::PointCloud<PointType>::Ptr downsampled_cloud(new pcl::PointCloud<PointType>);

        //다운샘플링을 수행
        sor.filter(*downsampled_cloud);
        
        double clusterTolerance = 0.3;
        int minSize = 3;
        int maxSize = 100;
        int clusterId = 1;
        int cone_count =0;

        if (downsampled_cloud->points.size() > 0){
            pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
            tree->setInputCloud(downsampled_cloud);
            std::vector<pcl::PointIndices> clusterIndices;
            pcl::EuclideanClusterExtraction<PointType> ec;
            ec.setClusterTolerance(clusterTolerance);
            ec.setMinClusterSize(minSize);
            ec.setMaxClusterSize(maxSize);
            ec.setSearchMethod(tree);
            ec.setInputCloud(downsampled_cloud);
            ec.extract(clusterIndices);

            for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin (); it != clusterIndices.end (); ++it)
            {
                pcl::CentroidPoint<PointType> centroid;
                
                for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                {
                    PointType pt = downsampled_cloud->points[*pit];
                    PointType pt2;

                    pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
                    centroid.add(pt2);
                }
                

                PointType c1;
                centroid.get(c1);
                double cone_distance = 0.0;

                cone_distance = ((c1.x - thisPose6D.x)*(c1.x - thisPose6D.x) + (c1.y - thisPose6D.y)*(c1.y - thisPose6D.y));

                //map 자료구조에 넣기 -> distance 순으로 나열됨
                if(cone_distance < 12*12) cone_center_point[cone_distance] = c1; 
            }       
        }
        // map의 모든 원소에 접근
        for(map<double,PointType>::const_iterator iter = cone_center_point.begin();iter != cone_center_point.end();iter++){//거리가 가장 가까운 물체부터
            save_coordinate(iter->second,mode);
        }
        cone_center_point.clear();
    }

    bool confirm_new_point( vector<PointType> cone_coordinate, PointType pointIn){
        const float distanceThreshold = 0.75; // 거리 임계값

        for(vector<PointType>::const_iterator iter = cone_coordinate.begin();iter != cone_coordinate.end();iter++){
            float distanceSquared = calculate_distance_squared(iter,pointIn);
             if (distanceSquared < (distanceThreshold * distanceThreshold)) {
                return false;
            }
        }
        
        return true;
    }
    
    bool confirm_new_point(const vector<PointType>& cone_coordinate, const vector<PointType>::const_iterator pointIn) {
        const float distanceThreshold = 0.75; // 거리 임계값

        for (const PointType& existingPoint : cone_coordinate) {
            float distanceSquared = calculate_distance_squared(pointIn, existingPoint);
            if (distanceSquared < (distanceThreshold * distanceThreshold)) {
                return false;
            }
        }

        return true;
    }

    void save_coordinate(PointType pointIn,int mode ){
        
        bool new_point = false;

        PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);

        if(mode ==0){
            if(cone_coordinate.size() ==0) {
                cone_coordinate.push_back(pointIn);
            }

            else{
                new_point = confirm_new_point(cone_coordinate,pointIn);

                if(new_point){    
                    cone_coordinate.push_back(pointIn);
                }
            }
        }
        if(mode ==1){
            if(barrel_coordinate.size() ==0) {
                barrel_coordinate.push_back(pointIn);
            }
            
            else{
                new_point = confirm_new_point(barrel_coordinate,pointIn);
                
                if(new_point){    
                    barrel_coordinate.push_back(pointIn);
                }
            }
        }

    }

    PointType find_closest_barrel(const PointType& current_position, const std::vector<PointType>& barrel_coordinate) {
        if (barrel_coordinate.empty()) {
            PointType tmp;
            tmp.x = 10000000000.0;
            tmp.y = 10000000000.0;
            tmp.z = 10000000000.0;
            return tmp; 
        }

        PointType closest_barrel = barrel_coordinate[0]; 
        float min_distance = calculate_distance(current_position, closest_barrel);

        
        for (const PointType& barrel : barrel_coordinate) {
            float distance = calculate_distance(current_position, barrel);
            if (distance < min_distance) {
                min_distance = distance;
                closest_barrel = barrel;
            }
        }

        return closest_barrel;
    }

    float calculate_distance(PointType point ,vector<PointType>::const_iterator cone_coordinate){
            float distance = sqrt((point.x-cone_coordinate->x)*(point.x-cone_coordinate->x)+(point.y-cone_coordinate->y)*(point.y-cone_coordinate->y));
            return distance; 
    }

     float calculate_distance(PointType point ,PointType point2){
            float distance = sqrt((point.x-point2.x)*(point.x-point2.x)+(point.y-point2.y)*(point.y-point2.y));
            return distance; 
    }
    float calculate_distance_squared(vector<PointType>::const_iterator cone_coordinate, const PointType& point ){
            float distance = (point.x-cone_coordinate->x)*(point.x-cone_coordinate->x)+(point.y-cone_coordinate->y)*(point.y-cone_coordinate->y);
            return distance; 
    }

     float calculate_distance_squared(PointType point ,PointType& point2){
            float distance = (point.x-point2.x)*(point.x-point2.x)+(point.y-point2.y)*(point.y-point2.y);
            return distance; 
    }

    void publishCloud(){
        clear_markerarrays();

        sensor_msgs::PointCloud2 laserCloudTemp;

        pcl::toROSMsg(*coneROICloud, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "base_link";
        pubConeROICloud.publish(laserCloudTemp);

        pcl::toROSMsg(*barrelROICloud, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "base_link";
        pubBarrelROICloud.publish(laserCloudTemp);

        visual_pose_info();
        pubCurrentEgo.publish(ego_markerarray);
        cout << "barrels:" <<barrel_coordinate.size() << endl;
        cout << "cones:" <<cone_coordinate.size() << endl;

        if(set_Uturn_position){
            visual_sign_info();
            pubUturnSign.publish(uturn);
        }
        if(cone_coordinate.size() > 1){
            visual_cone_info(cone_coordinate);
            pubCone_coordinate.publish(cone_markerarray);
        }
        if(barrel_coordinate.size() > 1){
            visual_closest_barrel_info(barrel_coordinate);
            pubBarrel_coordinate.publish(barrel_markerarray);
        }
    }

    void clear_markerarrays(){
        cone_markerarray.markers.clear();
        barrel_markerarray.markers.clear();
        ego_markerarray.markers.clear();
        uturn.markers.clear();
    }
    void publishTF(){

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(transformTobeMapped[2]);
    
        MappedTrans.stamp_ = ros::Time().fromSec(timeLaserOdometry);
        MappedTrans.setRotation(tf::Quaternion(odom_quat.x, odom_quat.y, odom_quat.z, odom_quat.w));
        MappedTrans.setOrigin(tf::Vector3(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]));
        
        tfBroadcaster.sendTransform(MappedTrans);
    }
    void visual_cone_info(vector<PointType> cone_coordinate){
        //visual marker
        visualization_msgs::Marker marker;
        int Id=0;

        for(vector<PointType>::const_iterator iter = cone_coordinate.begin();iter != cone_coordinate.end();iter++){
            
            marker.header.frame_id = "base_link";
            marker.header.stamp = ros::Time().now();
            marker.id = Id;

            marker.type = visualization_msgs::Marker::CUBE; 
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = iter->x;
            marker.pose.position.y = iter->y;
            marker.pose.position.z = iter->z;

            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
         
            marker.scale.x = 0.6;
            marker.scale.y = 0.6;
            marker.scale.z = 0.6;

            marker.color.r = 1;
            marker.color.g = 1;
            marker.color.b = 0;
            marker.color.a = 1;
            marker.lifetime = ros::Duration(0.5);
            
            cone_markerarray.markers.push_back(marker);
            
            Id++;
        }
    }

    void visual_closest_barrel_info(vector<PointType> barrel_coordinate){
        //visual marker
        visualization_msgs::Marker marker;
        PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
        PointType ego;
        ego.x = thisPose6D.x;
        ego.y = thisPose6D.y;
        ego.z = thisPose6D.z;

        PointType closest_barrel = find_closest_barrel(ego,barrel_coordinate);  

        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time().now();
        marker.id = 0;

        marker.type = visualization_msgs::Marker::SPHERE; 
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = closest_barrel.x;
        marker.pose.position.y = closest_barrel.y;
        marker.pose.position.z = closest_barrel.z;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = 0.6;
        marker.scale.y = 0.6;
        marker.scale.z = 0.6;
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;
        marker.color.a = 1;
        marker.lifetime = ros::Duration(0.5);
        
        barrel_markerarray.markers.push_back(marker);
            
    }

    void visual_pose_info(){
        //ego marker
        visualization_msgs::Marker marker;

        PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
        geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw
                                  (transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time::now();
        marker.id = 0;
        
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = thisPose6D.x;
        marker.pose.position.y = thisPose6D.y;
        marker.pose.position.z = thisPose6D.z;
  
        marker.pose.orientation.x = geoQuat.x;
        marker.pose.orientation.y = geoQuat.y;
        marker.pose.orientation.z = geoQuat.z;
        marker.pose.orientation.w = geoQuat.w;
        
        marker.scale.x = lidar_to_gps_D;
        marker.scale.y = 0.4;
        marker.scale.z = 0.4;
        
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.7;
        ego_markerarray.markers.push_back(marker);
    }
    void visual_sign_info(){
        //ego marker
        visualization_msgs::Marker marker;

        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time::now();
        marker.id = 0;
        
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
  
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = 0.4;
        marker.scale.y = 0.4;
        marker.scale.z = 0.4;
        
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.7;

        uturn.markers.push_back(marker);
    }
};


int main(int argc, char** argv){

    ros::init(argc, argv, "u_turn");
    
    U_turn CD;

    ROS_INFO("\033[1;32m---->\033[0m Uturn Started.");

    ros::spin();
    return 0;
}
