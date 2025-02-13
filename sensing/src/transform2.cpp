#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>



geometry_msgs::PoseStamped current_pose;

void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    current_pose = *msg;

    // geometry_msgs::PoseStamped waypoint;
    // waypoint.header.stamp = ros::Time::now();
    // waypoint.header.frame_id = "base_link";
    

    // waypoint.pose.position.x = current_pose.pose.position.x;//relative_translation.x();
    // waypoint.pose.position.y = current_pose.pose.position.y;//relative_translation.y();
    // waypoint.pose.position.z = 0.0;//relative_translation.z();
    // waypoint.pose.orientation.x = 0.0;//relative_rotation.x();
    // waypoint.pose.orientation.y = 0.0;//relative_rotation.y();
    // waypoint.pose.orientation.z = 0.0;//relative_rotation.z();
    // waypoint.pose.orientation.w = 0.0;//relative_rotation.w();

    // trajectory.poses.push_back(waypoint);
}


int main(int argc, char** argv){
    ros::init(argc, argv, "odometry_publisher");
    ROS_INFO("\033[1;32m---->\033[0m Waypoint Tracking Started.");

    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    ros::Subscriber pose_sub = n.subscribe<geometry_msgs::PoseStamped>("/local_msg", 1, poseCallback);

    tf::TransformBroadcaster odom_broadcaster;

    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    double vx = 0.1;
    double vy = -0.1;
    double vth = 0.1;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    while(n.ok()){

        ros::spinOnce();              
        current_time = ros::Time::now();

        double dt = (current_time - last_time).toSec();
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        double angle_rad = 90 * M_PI / 180.0;
        // tf::Quaternion quaternion;
        // quaternion.setRPY(0, 0, angle_rad);
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(angle_rad);

        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "base_link";
        odom_trans.child_frame_id = "base_laser";
        // for (float i=0; i<100;++i){
        odom_trans.transform.translation.x = current_pose.pose.position.x;
        odom_trans.transform.translation.y = current_pose.pose.position.y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
        odom_broadcaster.sendTransform(odom_trans);
        // }
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        // for (float i=0; i<100;++i){
        //     odom.pose.pose.position.x = i/10;
        //     odom.pose.pose.position.y = i/10;
        //     odom.pose.pose.position.z = 0.0;
        //     odom.pose.pose.orientation = odom_quat;

        // }

        odom.child_frame_id = "base_laser";
        odom.twist.twist.linear.x = 0;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.angular.z = 0;

        odom_pub.publish(odom);

        last_time = current_time;
    }
}