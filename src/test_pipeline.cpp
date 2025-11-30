#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <random>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_pipeline");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::TwistStamped>("/test_cmd_vel_quadruped", 10);

    // random generator for yaw rate in (-1, 1)
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    std::uniform_real_distribution<double> dist_lin(-0.3, 0.3);

    double yaw_rate = dist(gen);
    double lin_x = dist_lin(gen);
    double lin_y = dist_lin(gen);
    ros::Time last_change = ros::Time::now();

    ros::Rate rate(50); // publish at 50 Hz
    ROS_WARN("test_pipeline: starting publisher on /cmd_vel_quadruped; initial yaw_rate=%.3f", yaw_rate);

    while (ros::ok()) {
        ros::Time now = ros::Time::now();
        if ((now - last_change) >= ros::Duration(1.0)) {
            yaw_rate = dist(gen);
            lin_x = dist_lin(gen);
            lin_y = dist_lin(gen);
            last_change = now;
            ROS_INFO("test_pipeline: new yaw_rate=%.3f, lin_x=%.3f, lin_y=%.3f", yaw_rate, lin_x, lin_y);
        }

        geometry_msgs::TwistStamped msg;
        msg.header.stamp = now;
        msg.header.frame_id = "quadruped";
        msg.twist.linear.x = lin_x;
        msg.twist.linear.y = lin_y;
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
