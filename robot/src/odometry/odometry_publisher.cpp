//
// Created by zdy on 19-5-20.
//

#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "rclcpp/clock.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);


    auto node = rclcpp::Node::make_shared("odometry_publisher");

    rmw_qos_profile_t qos_profile;
    qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    qos_profile.depth = 1;
    qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

    auto odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("odom", qos_profile);

    tf2_ros::TransformBroadcaster odom_broadcaster(node);

    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    double vx = 0.1;
    double vy = -0.1;
    double vth = 0.1;


    rclcpp::TimeSource ts(node);
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    ts.attachClock(clock);

    rclcpp::Time current_time, last_time;
    current_time = clock->now();
    last_time = clock->now();

    rclcpp::WallRate loop_rate(100);


    while (rclcpp::ok()) {
        current_time = clock->now();

        //compute odometry in a typical way given the velocities of the robot
        double dt = (current_time - last_time).seconds();
        double delta_x = (vx * std::cos(th) - vy * std::sin(th)) * dt;
        double delta_y = (vx * std::sin(th) + vy * std::cos(th)) * dt;
        double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        tf2::Quaternion quat = tf2::Quaternion();
        quat.setRPY(0, 0, th);
        geometry_msgs::msg::Quaternion odom_quat;
        odom_quat.x = quat.x();
        odom_quat.y = quat.y();
        odom_quat.z = quat.z();
        odom_quat.w = quat.w();

        //first, we'll publish the transform over tf
        geometry_msgs::msg::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        //publish the message
        odom_pub->publish(odom);

        last_time = current_time;

        rclcpp::spin_some(node);

        loop_rate.sleep();
    }
}