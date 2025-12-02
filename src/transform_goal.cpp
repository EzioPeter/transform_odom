#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <std_msgs/Int16.h>
#include <atomic>

using namespace std;

static std::atomic<int16_t> g_go_flag{0};

tf::TransformListener* listener_ptr;
ros::Publisher cmd_vel_quadruped_pub;

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

void cmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr& cmd_msg)
{
    // Expect cmd_msg->header.frame_id == "world" (global cmd)
    // Transform linear velocity and angular (yaw_dot) into 'quadruped' frame
    geometry_msgs::Vector3Stamped lin_world, lin_quad;
    geometry_msgs::Vector3Stamped ang_world, ang_quad;

    // prepare headers: prefer cmd_msg timestamp but guard against future stamps
    ros::Time stamp = cmd_msg->header.stamp;
    if (stamp.isZero()) stamp = ros::Time::now();
    ros::Time now = ros::Time::now();
    // clamp small future timestamps to now to avoid extrapolation
    if (stamp > now + ros::Duration(0.01))
      stamp = now;

    lin_world.header.frame_id = cmd_msg->header.frame_id.empty() ? std::string("world") : cmd_msg->header.frame_id;
    lin_world.header.stamp = stamp;
    lin_world.vector.x = cmd_msg->velocity.x;
    lin_world.vector.y = cmd_msg->velocity.y;
    lin_world.vector.z = cmd_msg->velocity.z;

    // yaw_dot is a scalar around z axis in world frame — place into vector z
    ang_world.header = lin_world.header;
    ang_world.vector.x = 0.0;
    ang_world.vector.y = 0.0;
    ang_world.vector.z = cmd_msg->yaw_dot;

    bool transformed = false;
    try {
        listener_ptr->transformVector("quadruped", lin_world, lin_quad);
        listener_ptr->transformVector("quadruped", ang_world, ang_quad);
        transformed = true;
    }
    catch (tf::TransformException &ex) {
        ROS_DEBUG("[transform_goal] TF transform failed (stamp %f): %s; will retry with latest transform.", stamp.toSec(), ex.what());
    }

    if (!transformed) {
        // fallback: use latest available transform (time = 0)
        lin_world.header.stamp = ros::Time(0);
        ang_world.header.stamp = ros::Time(0);
        try {
            listener_ptr->transformVector("quadruped", lin_world, lin_quad);
            listener_ptr->transformVector("quadruped", ang_world, ang_quad);
            transformed = true;
            ROS_DEBUG("[transform_goal] used latest TF (time=0) as fallback");
        } catch (tf::TransformException &ex) {
            ROS_WARN("[transform_goal] TF transform error when converting cmd (fallback): %s", ex.what());
        }
    }

    int16_t go_val = g_go_flag.load();

    if (transformed && go_val) {
        geometry_msgs::Twist twist_msg;
        twist_msg.linear = lin_quad.vector;
        twist_msg.angular = ang_quad.vector;
        // enforce zero for z linear and x/y angular components
        twist_msg.linear.z = 0.0;
        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;

        cmd_vel_quadruped_pub.publish(twist_msg);
    }
}

void goFlagCallback(const std_msgs::Int16::ConstPtr &msg)
{
    g_go_flag.store(msg->data);
    ROS_DEBUG("Received go_flag = %d", msg->data);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "transform_goal");
    ros::NodeHandle nh;

    tf::TransformListener listener(nh);
    listener_ptr = &listener;

    ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 10, goalCallback);
    cmd_vel_quadruped_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_quadruped", 10);
    ros::Subscriber cmd_sub = nh.subscribe<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 10, cmdCallback);
    ros::Subscriber go_flag_sub = nh.subscribe<std_msgs::Int16>("/ego_planner_node/go_flag", 10, goFlagCallback);
    // also subscribe to planner's pos_cmd in case planner publishes there
    // ros::Subscriber cmd_sub2 = nh.subscribe<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 10, cmdCallback);

    ros::spin(); 
    return 0;
}