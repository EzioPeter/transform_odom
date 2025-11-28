#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <tf/transform_broadcaster.h>
#include <cmath>  
 
Eigen::Vector3d pos;
Eigen::Quaterniond quaternion;
 
void odom_src_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if(msg->header.frame_id == "world")
    {
        pos = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
 
        quaternion = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        Eigen::Quaterniond rot_z(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ()));
        quaternion = rot_z * quaternion;

        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(pos(0), pos(1), pos(2)));
        transform.setRotation(tf::Quaternion(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "quadruped"));
    }
}
 
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "transform_odom");
    ros::NodeHandle nh("~");
 
    ros::Subscriber slam_sub = nh.subscribe<nav_msgs::Odometry>("odom_src", 100, odom_src_callback);
 
    ros::Publisher odom_tar_pub = nh.advertise<geometry_msgs::PoseStamped>("odom_tar", 10);
 
 
    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(100.0);
 
    ros::Time last_request = ros::Time::now();
 
    while(ros::ok()){
        geometry_msgs::PoseStamped odom_tar;
 
        odom_tar.pose.position.x = pos[0];
        odom_tar.pose.position.y = pos[1];
        odom_tar.pose.position.z = pos[2];
 
        odom_tar.pose.orientation.x = quaternion.x();
        odom_tar.pose.orientation.y = quaternion.y();
        odom_tar.pose.orientation.z = quaternion.z();
        odom_tar.pose.orientation.w = quaternion.w();
 
        odom_tar.header.stamp = ros::Time::now();
        odom_tar.header.frame_id = "world";
        odom_tar_pub.publish(odom_tar);
 
        ros::spinOnce();
        rate.sleep();
    }
 
    return 0;
}