#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

tf::TransformListener* listener_ptr;

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_msg)
{
    if (goal_msg->header.frame_id != "world") {
        ROS_WARN("The goal frame is not world, please check the goal frame");
        return;
    }

    geometry_msgs::PoseStamped goal_quadruped;

    try {
        listener_ptr->waitForTransform("quadruped", "world", 
                                      goal_msg->header.stamp, ros::Duration(1.0));
        listener_ptr->transformPose("quadruped", *goal_msg, goal_quadruped);

        ROS_INFO("=== world tf axis ===");
        ROS_INFO("Position: (%.4f, %.4f, %.4f)", 
                 goal_msg->pose.position.x, goal_msg->pose.position.y, goal_msg->pose.position.z);
        ROS_INFO("Quaternion: (x=%.4f, y=%.4f, z=%.4f, w=%.4f)",
                 goal_msg->pose.orientation.x, goal_msg->pose.orientation.y,
                 goal_msg->pose.orientation.z, goal_msg->pose.orientation.w);

        ROS_INFO("=== quadruped tf axis ===");
        ROS_INFO("Position: (%.4f, %.4f, %.4f)", 
                 goal_quadruped.pose.position.x, goal_quadruped.pose.position.y, goal_quadruped.pose.position.z);
        ROS_INFO("Quaternion: (x=%.4f, y=%.4f, z=%.4f, w=%.4f)",
                 goal_quadruped.pose.orientation.x, goal_quadruped.pose.orientation.y,
                 goal_quadruped.pose.orientation.z, goal_quadruped.pose.orientation.w);

    } catch (tf::TransformException& ex) {
        ROS_ERROR("Transform Error: %s", ex.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "transform_goal");
    ros::NodeHandle nh;

    tf::TransformListener listener(nh);
    listener_ptr = &listener;

    ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 10, goalCallback);

    ros::spin(); 
    return 0;
}