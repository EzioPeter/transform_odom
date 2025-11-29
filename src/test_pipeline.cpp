#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <random>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_pipeline");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::TwistStamped>("/cmd_vel_quadruped", 10);

    // random generator for yaw rate in (-1, 1)
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(-1.0, 1.0);

    double yaw_rate = dist(gen);
    ros::Time last_change = ros::Time::now();

    ros::Rate rate(50); // publish at 50 Hz
    ROS_WARN("test_pipeline: starting publisher on /cmd_vel_quadruped; initial yaw_rate=%.3f", yaw_rate);

    while (ros::ok()) {
        ros::Time now = ros::Time::now();
        if ((now - last_change) >= ros::Duration(5.0)) {
            yaw_rate = dist(gen);
            last_change = now;
            ROS_INFO("test_pipeline: new yaw_rate=%.3f", yaw_rate);
        }

        geometry_msgs::TwistStamped msg;
        msg.header.stamp = now;
        msg.header.frame_id = "quadruped";
        msg.twist.linear.x = 0.0;
        msg.twist.linear.y = 0.0;
        msg.twist.linear.z = 0.0;

        msg.twist.angular.x = 0.0;
        msg.twist.angular.y = 0.0;
        msg.twist.angular.z = yaw_rate;

        pub.publish(msg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
