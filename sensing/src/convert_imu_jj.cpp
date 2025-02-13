#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include <math.h>
using namespace std;



class Imu_Convertor{
public:
    ros::NodeHandle nh;
    ros::Publisher imu_pub;
    ros::Publisher trajectory_pub;
    ros::Publisher odometry_pub;
    ros::Publisher gps_pub;
    ros::Subscriber imu_sub;
    nav_msgs::Path trajectory;
    bool start = false;

    Imu_Convertor(){
        imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/quaternion", 10);
        trajectory.header.stamp = ros::Time::now();
        trajectory.header.frame_id = "map";
        trajectory_pub = nh.advertise<nav_msgs::Path>("/erp_trajectory", 10);
        odometry_pub = nh.advertise<nav_msgs::Odometry>("/jaejun_odometry", 10);
        gps_pub = nh.advertise<nav_msgs::Odometry>("/jaejun_gps", 10);
        imu_sub = nh.subscribe("/local_msgs_to_vision", 10, &Imu_Convertor::imuCallback, this);
    }


    void imuCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){

        double yaw = msg->pose.orientation.w - 102.14;
        // double yaw = M_PI/2 + theta;
        double cr = cos(0);
        double sr = sin(0);
        double cp = cos(0);
        double sp = sin(0);
        double cy = cos(yaw*0.5);
        double sy = sin(yaw*0.5);
        
        double ori_w = cr * cp * cy + sr * sp * sy;
        double ori_x = sr * cp * cy - cr * sp * sy;
        double ori_y = cr * sp * cy + sr * cp * sy;
        double ori_z = cr * cp * sy - sr * sp * cy;   

        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.orientation.x = ori_x;
        imu_msg.orientation.y = ori_y;
        imu_msg.orientation.z = ori_z;
        imu_msg.orientation.w = ori_w;


        geometry_msgs::PoseStamped waypoint;
        waypoint.header.stamp = ros::Time::now();
        waypoint.header.frame_id = "map";
        if (start == false){
            
        }
        double yaw2 = -293.20 - 3.0;
        double radian = yaw2 * M_PI / 180.0;  
        double x = msg->pose.position.x - 178.91;
        double y = msg->pose.position.y - 0.68;
        double newX = x * cos(radian) - y * sin(radian);
        double newY = x * sin(radian) + y * cos(radian);
        x = newX;
        y = newY;

        waypoint.pose.position.x = x;//relative_translation.x();
        waypoint.pose.position.y = y;//relative_translation.y();
        waypoint.pose.position.z = 0.0;//relative_translation.z();

        waypoint.pose.orientation.x = ori_x;//relative_rotation.x();
        waypoint.pose.orientation.y = ori_y;//relative_rotation.y();
        waypoint.pose.orientation.z = ori_z;//relative_rotation.z();
        waypoint.pose.orientation.w = ori_w;//relative_rotation.w();
        
        trajectory.poses.push_back(waypoint);

        //#########################jaejun odometry######################
        nav_msgs::Odometry odometry;
        odometry.pose.pose.position.x = x;
        odometry.pose.pose.position.y = y;
        odometry.pose.pose.position.z = 0.0;
        odometry.pose.pose.orientation.x = ori_x;
        odometry.pose.pose.orientation.y = ori_y;
        odometry.pose.pose.orientation.z = ori_z;
        odometry.pose.pose.orientation.w = ori_w;


        odometry_pub.publish(odometry);
        trajectory_pub.publish(trajectory);
        imu_pub.publish(imu_msg);
        cout << "hhhhhhhhh"<<endl;
        // return std::make_tuple(ori_x, ori_y, ori_z, ori_w);
        // covariance = [
        //             .01, 0, 0, 0, 0, 0, 0, .01, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        //             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, .0001
        //         ]
        //jaejun_gps
        nav_msgs::Odometry gps;
        gps.pose.pose.position.x = x;
        gps.pose.pose.position.y = y;
        gps.pose.pose.position.z = 0.0;
        gps.pose.pose.orientation.x = ori_x;
        gps.pose.pose.orientation.y = ori_y;
        gps.pose.pose.orientation.z = ori_z;
        gps.pose.pose.orientation.w = ori_w;    

        for (int i = 0; i < 36; ++i) {
            gps.pose.covariance[i] = 0.001;
        }
        gps_pub.publish(gps);        
    }
    
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_conversion_node");

    Imu_Convertor imu_con;

    cout << "converting imu data ...."<<endl;
    ros::spin();

    return 0;
}