#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <cmath>  
 
Eigen::Vector3d pos;
Eigen::Quaterniond quaternion;
geometry_msgs::Twist saved_twist;

void odom_src_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if(msg->header.frame_id == "world")
    {
        pos = Eigen::Vector3d(0.0, 0.0, 0.0);
 
        quaternion = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
        Eigen::Quaterniond rot_y(Eigen::AngleAxisd(-M_PI/12, Eigen::Vector3d::UnitY()));
        quaternion = rot_y * quaternion;
        // save incoming velocity so we can forward it in odom_tar
        saved_twist = msg->twist.twist;
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(pos(0), pos(1), pos(2)));
        transform.setRotation(tf::Quaternion(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "body", "quadruped"));
    }
}
 
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "transform_odom");
    ros::NodeHandle nh("~");
 
    ros::Subscriber slam_sub = nh.subscribe<nav_msgs::Odometry>("odom_src", 100, odom_src_callback);
 
    ros::Publisher odom_tar_pub = nh.advertise<nav_msgs::Odometry>("odom_tar", 10);
 
 
    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(100.0);
 
    ros::Time last_request = ros::Time::now();
 
    while(ros::ok()){
        nav_msgs::Odometry odom_tar;

        odom_tar.pose.pose.position.x = pos[0];
        odom_tar.pose.pose.position.y = pos[1];
        odom_tar.pose.pose.position.z = pos[2];

        odom_tar.pose.pose.orientation.x = quaternion.x();
        odom_tar.pose.pose.orientation.y = quaternion.y();
        odom_tar.pose.pose.orientation.z = quaternion.z();
        odom_tar.pose.pose.orientation.w = quaternion.w();

        // copy velocity from the latest received odom_src
        odom_tar.twist.twist = saved_twist;

        odom_tar.header.stamp = ros::Time::now();
        odom_tar.header.frame_id = "world";
        odom_tar_pub.publish(odom_tar);
 
        ros::spinOnce();
        rate.sleep();
    }
 
    return 0;
}