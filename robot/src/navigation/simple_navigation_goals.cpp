//
// Created by zdy on 19-5-21.
//

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "rclcpp/clock.hpp"
#include <rclcpp_action/create_client.hpp>
#include "tf2/LinearMath/Quaternion.h"
//#include "actionlib_msgs/msg/goal_status.hpp"
#include "action_msgs/msg/goal_status.hpp"
#include "robot/msg/move_base_action_goal.hpp"

#include <boost/thread.hpp>


int main(int argc, char **argv) {

    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("simple_navigation_goals");

    rclcpp_action::Client<robot::msg::MoveBaseActionGoal>::SharedPtr action_client = rclcpp_action::create_client<robot::msg::MoveBaseActionGoal>(node, "pose_base_controller");


    //give some time for connections to register
    sleep(2.0);


    rclcpp::TimeSource ts(node);
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    ts.attachClock(clock);


    tf2::Quaternion quat = tf2::Quaternion();
    quat.setRPY(0, 0, M_PI);

    robot::msg::MoveBaseActionGoal goal;

    //we'll send a goal to the robot to move 2 meters forward
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = clock->now();


    goal.target_pose.pose.position.x = 2.0;
    goal.target_pose.pose.position.y = 0.2;
    goal.target_pose.pose.orientation = quat;

    RCLCPP_INFO(node->get_logger(), "Sending goal");
    auto result = action_client->async_send_goal(goal);
    action_client->


//    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){

    if (rclcpp::spin_until_future_complete(node, result)  == rclcpp::executor::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), result.get());
        RCLCPP_INFO(node->get_logger(), "Hooray, the base moved 2 meters forward");
    } else {
        RCLCPP_INFO(node->get_logger(), "The base failed to move forward 2 meters for some reason");
    }


    rclcpp::spin(node);

    return 0;
}