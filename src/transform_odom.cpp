#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
 
 
Eigen::Vector3d p_tar;
Eigen::Quaterniond q_tar;
 
void odom_src_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if(msg->header.frame_id == "world")
    {
        p_tar = Eigen::Vector3d(-msg->pose.pose.position.y, -msg->pose.pose.position.x, msg->pose.pose.position.z);
 
        q_tar = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

        // Eigen::AngleAxisd roll(0,Eigen::Vector3d::UnitX()); 
        // Eigen::AngleAxisd pitch(0,Eigen::Vector3d::UnitY());
        // Eigen::AngleAxisd yaw(M_PI/2,Eigen::Vector3d::UnitZ());    // 绕 z 轴旋转 pi / 2

        // Eigen::Quaterniond _q_tar = roll * pitch * yaw;
        // q_tar = q_tar * _q_tar;

        Eigen::Vector3d ypr;
        ypr(2) = atan2(2 * (q_tar.w() * q_tar.x() + q_tar.y() * q_tar.z()), 1 - 2 * (q_tar.x() * q_tar.x() + q_tar.y() * q_tar.y()));
        ypr(1) = asin(2 * (q_tar.w() * q_tar.y() - q_tar.z() * q_tar.x()));
        ypr(0) = atan2(2 * (q_tar.w() * q_tar.z() + q_tar.x() * q_tar.y()), 1 - 2 * (q_tar.y() * q_tar.y() + q_tar.z() * q_tar.z()));

        q_tar = Eigen::AngleAxisd(ypr(0), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(-ypr(2), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(-ypr(1), Eigen::Vector3d::UnitX());
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
 
        odom_tar.pose.position.x = p_tar[0];
        odom_tar.pose.position.y = p_tar[1];
        odom_tar.pose.position.z = p_tar[2];
 
        odom_tar.pose.orientation.x = q_tar.x();
        odom_tar.pose.orientation.y = q_tar.y();
        odom_tar.pose.orientation.z = q_tar.z();
        odom_tar.pose.orientation.w = q_tar.w();
 
        odom_tar.header.stamp = ros::Time::now();
        odom_tar.header.frame_id = "world";
        odom_tar_pub.publish(odom_tar);
 
        // ROS_INFO("\nposition:\n   x: %.18f\n   y: %.18f\n   z: %.18f\norientation:\n   x: %.18f\n   y: %.18f\n   z: %.18f\n   w: %.18f", \
        // p_tar[0],p_tar[1],p_tar[2],q_tar.x(),q_tar.y(),q_tar.z(),q_tar.w());
 
        ros::spinOnce();
        rate.sleep();
    }
 
    return 0;
}