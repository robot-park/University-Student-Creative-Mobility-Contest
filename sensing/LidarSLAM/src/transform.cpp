#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <json/json.h>
#include <fstream>

#pragma comment(lib , "json/json_vc71_libmtd")

geometry_msgs::PoseStamped current_pose;
nav_msgs::Path trajectory;

void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    current_pose = *msg;

    geometry_msgs::PoseStamped waypoint;
    waypoint.header.stamp = ros::Time::now();
    waypoint.header.frame_id = "base_link";
    

    waypoint.pose.position.x = current_pose.pose.position.x;//relative_translation.x();
    waypoint.pose.position.y = current_pose.pose.position.y;//relative_translation.y();
    waypoint.pose.position.z = 0.0;//relative_translation.z();
    waypoint.pose.orientation.x = 0.0;//relative_rotation.x();
    waypoint.pose.orientation.y = 0.0;//relative_rotation.y();
    waypoint.pose.orientation.z = 0.0;//relative_rotation.z();
    waypoint.pose.orientation.w = 0.0;//relative_rotation.w();

    trajectory.poses.push_back(waypoint);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "robotz_tf_publisher");
    ROS_INFO("\033[1;32m---->\033[0m Waypoint Tracking Started.");

    ros::NodeHandle n;
    ros::Rate r(0.1);

    tf::TransformBroadcaster broadcaster;
    tf::TransformBroadcaster broadcaster1;

    ros::Publisher trajectory_pub = n.advertise<nav_msgs::Path>("/erp_trajectory", 10);
    ros::Publisher global_path_pub = n.advertise<visualization_msgs::Marker>("/gloabl_path",10);
    // ros::Publisher lane_pub = n.advertise<visualization_msgs::Marker>("/lane",10);
    ros::Subscriber pose_sub = n.subscribe<geometry_msgs::PoseStamped>("/local_msg", 1, poseCallback);

    while (n.ok()) {
        
        visualization_msgs::Marker global_path;
        visualization_msgs::Marker lane;

        traje+
        ctory.header.stamp = ros::Time::now();
        trajectory.header.frame_id = "base_link";

        Json::Value root;
        Json::Reader reader;
        std::ifstream t;
        std::string index;
        // t.open("/home/vision/lidar_ws/src/jaejun/maps/begin_map.json");
        t.open("/home/vision/lidar_ws/src/jaejun/maps/songdo_map.json");
        reader.parse(t, root);
        t.close();

        global_path.scale.x = 0.35;
        global_path.scale.y = 0.35;
        global_path.color.a = 1.0;
        global_path.color.r = 1.0;
        global_path.action = visualization_msgs::Marker::ADD;
        global_path.type = visualization_msgs::Marker::POINTS;
        global_path.header.stamp = ros::Time::now();
        global_path.header.frame_id = "base_link";
        
        // lane.scale.x = 0.08;
        // lane.scale.y = 0.08;
        // lane.color.a = 1.0;
        // lane.color.g = 1.0;
        // lane.action = visualization_msgs::Marker::ADD;
        // lane.type = visualization_msgs::Marker::POINTS;
        // lane.header.stamp = ros::Time::now();
        // lane.header.frame_id = "base_link";


        for (int k = 0; k < root.size(); ++k) {
            if (k%10 == 0){
                index = std::to_string(k);
                double x = root[index][0].asDouble();
                double y = root[index][1].asDouble();
                double llane_x, llane_y, rlane_x, rlane_y, road_vector, len;
                geometry_msgs::Point node;
                geometry_msgs::Point node_r;
                geometry_msgs::Point node_l;
                node.x = x;
                node.y = y;
                node.z = 0.0;
                
                // if (global_path.points.size() > 0){
                //     road_vector = atan2(fabs(y-global_path.points[-1].y), fabs(x-global_path.points[-1].x));
                //     len = std::hypot(y-global_path.points[-1].y, x-global_path.points[-1].x);
                //     node_l.x = x + len*cos(road_vector) + 2*cos(90-road_vector);
                //     node_l.y = y + len*sin(road_vector) - 2*sin(90-road_vector);
                //     node_l.z = 0.0;
                //     node_r.x = x + len*cos(road_vector) - 2*cos(90-road_vector);
                //     node_r.y = y + len*sin(road_vector) + 2*sin(90-road_vector);
                //     node_r.z = 0.0;
                //     lane.points.push_back(node_l);
                //     lane.points.push_back(node_r);
                // }
                global_path.points.push_back(node);
            }
        }
        global_path_pub.publish(global_path);
        // lane_pub.publish(lane);

        double angle_deg = current_pose.pose.orientation.w; // Ã¬Å¾â€¦Ã« Â¥ ÃªÂ°ÂÃ«Ââ€ž (0Ã¬â€”ÂÃ¬â€žÅ“ 360Ã«Ââ€ž)
        double angle_rad = angle_deg * M_PI / 180.0;
        tf::Quaternion quaternion;
        quaternion.setRPY(0, 0, angle_rad);
        // tf::StampedTransform transform(tf::Transform(tf::Quaternion(0,0,0, 1), tf::Vector3(current_pose.pose.position.x , current_pose.pose.position.y, 0.0)),
        //     ros::Time::now(), "base_link", "base_laser");
        tf::StampedTransform transform(tf::Transform(tf::Quaternion(quaternion.x(), quaternion.y(),quaternion.z(), 1), tf::Vector3(current_pose.pose.position.x , current_pose.pose.position.y, 0.0)),
            ros::Time::now(), "base_link", "base_laser");    

        broadcaster.sendTransform(transform);

        // geometry_msgs::PoseStamped waypoint;
        // waypoint.header.stamp = ros::Time::now();
        // waypoint.header.frame_id = "base_link";
        // // base_laser ÃªÂ·Â¸Ã«Å¸Â¬Ã«â€¹Ë†ÃªÂ¹Å’ erp Ã¬â€”ÂÃ¬â€žÅ“Ã¬ÂËœ base_linkÃ¬Æ’Â Ã¬ â€žÃ¬Â²Â´ mapÃ¬Æ’ÂÃ¬â€”ÂÃ¬â€žÅ“Ã¬ÂËœ Ã¬Æ’ÂÃ«Å’â‚¬Ã¬ ÂÃ¬ÂÂ¸ ÃªÂ¶Â¤Ã¬ Â
        // tf::Transform relative_transform = transform.inverse();
        // tf::Vector3 relative_translation = relative_transform.getOrigin();
        // tf::Quaternion relative_rotation = relative_transform.getRotation();

        // waypoint.pose.position.x = current_pose.pose.position.x;//relative_translation.x();
        // waypoint.pose.position.y = current_pose.pose.position.y;//relative_translation.y();
        // waypoint.pose.position.z = 0.0;//relative_translation.z();
        // waypoint.pose.orientation.x = 0.0;//relative_rotation.x();
        // waypoint.pose.orientation.y = 0.0;//relative_rotation.y();
        // waypoint.pose.orientation.z = 0.0;//relative_rotation.z();
        // waypoint.pose.orientation.w = 0.0;//relative_rotation.w();

        // trajectory.poses.push_back(waypoint);

        // }
        trajectory_pub.publish(trajectory);
        ros::spinOnce();
    }

    return 0;
}