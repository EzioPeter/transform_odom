#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <cmath>  

tf::TransformListener* tf_listener;
ros::Publisher imagine_tar_pub;

void imagine_src_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if(msg->header.frame_id == "body")
    {
        if(msg->pose.position.x != 0.0 || msg->pose.position.y != 0.0){
            geometry_msgs::PoseStamped pose_body_msg = *msg;
            geometry_msgs::PoseStamped pose_world_msg;
            try {
                tf_listener->waitForTransform("world", "body", pose_body_msg.header.stamp, ros::Duration(1.0));
                tf_listener->transformPose("world", pose_body_msg, pose_world_msg);
            } catch (tf::TransformException &ex) {
                ROS_WARN("Imagine cmd TF transformPose failed: %s", ex.what());
                return;
            }

            pose_world_msg.header.stamp = ros::Time::now();
            pose_world_msg.header.frame_id = "world";
            imagine_tar_pub.publish(pose_world_msg);
        }
    }
}
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "imagine_cmd");
    ros::NodeHandle nh("~");

    static tf::TransformListener listener(nh);
    tf_listener = &listener;
 
    ros::Subscriber imagine_src_sub = nh.subscribe<geometry_msgs::PoseStamped>("imagine_src", 100, imagine_src_callback);
 
    imagine_tar_pub = nh.advertise<geometry_msgs::PoseStamped>("imagine_tar", 10);
 
    ros::spin();
 
    return 0;
}