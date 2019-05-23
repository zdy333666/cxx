//
// Created by zdy on 19-5-21.
//

#include <inttypes.h>
#include <memory>
#include "example_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
// TODO(jacobperron): Remove this once it is included as part of 'rclcpp.hpp'
#include "rclcpp_action/rclcpp_action.hpp"

using Fibonacci = example_interfaces::action::Fibonacci;

rclcpp::Node::SharedPtr node = nullptr;


void feedback_callback(
        rclcpp_action::ClientGoalHandle<Fibonacci>::SharedPtr,
        const std::shared_ptr<const Fibonacci::Feedback> feedback) {

//    RCLCPP_INFO(g_node->get_logger(), "Next number in sequence received: %" PRId64, feedback->sequence.back());

    auto &sequence = feedback.get()->sequence;

    std::cout << "feedback: ";
    for (auto number : sequence) {
        std::cout << number << " ";
    }
    std::cout << std::endl;
}


int main(int argc, char **argv) {

    rclcpp::init(argc, argv);

    node = rclcpp::Node::make_shared("minimal_action_client");
    auto action_client = rclcpp_action::create_client<Fibonacci>(node, "fibonacci");

    if (!action_client->wait_for_action_server(std::chrono::seconds(20))) {
        RCLCPP_ERROR(node->get_logger(), "Action server not available after waiting");

        return 1;
    }

    // Populate a goal
    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = 10;

    RCLCPP_INFO(node->get_logger(), "Sending goal");



    // Ask server to achieve some goal and wait until it's accepted
    auto goal_handle_future = action_client->async_send_goal(goal_msg, feedback_callback);

    if (rclcpp::spin_until_future_complete(node, goal_handle_future) != rclcpp::executor::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "send goal call failed :(");

        return 1;
    }


    rclcpp_action::ClientGoalHandle<Fibonacci>::SharedPtr goal_handle = goal_handle_future.get();

    if (!goal_handle) {
        RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");

        return 1;
    }


    // Wait for the server to be done with the goal
    auto result_future = goal_handle->async_result();

    auto wait_result = rclcpp::spin_until_future_complete(node, result_future, std::chrono::seconds(3));

    if (rclcpp::executor::FutureReturnCode::TIMEOUT == wait_result) {

        RCLCPP_INFO(node->get_logger(), "canceling goal");

        // Cancel the goal since it is taking too long
        auto cancel_result_future = action_client->async_cancel_goal(goal_handle);

        if (rclcpp::spin_until_future_complete(node, cancel_result_future) != rclcpp::executor::FutureReturnCode::SUCCESS) {

            RCLCPP_ERROR(node->get_logger(), "failed to cancel goal");
            rclcpp::shutdown();

            return 1;
        }

        RCLCPP_INFO(node->get_logger(), "goal is being canceled");

    } else if (rclcpp::executor::FutureReturnCode::SUCCESS != wait_result) {

        RCLCPP_ERROR(node->get_logger(), "failed to get result");
        rclcpp::shutdown();

        return 1;
    }


    RCLCPP_INFO(node->get_logger(), "Waiting for result");

    if (rclcpp::spin_until_future_complete(node, result_future) != rclcpp::executor::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "get result call failed :(");

        return 1;
    }


    rclcpp_action::ClientGoalHandle<Fibonacci>::Result result = result_future.get();

    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(node->get_logger(), "Goal was aborted");
            return 1;
        case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(node->get_logger(), "Goal was canceled");
            return 1;
        default:
        RCLCPP_ERROR(node->get_logger(), "Unknown result code");
            return 1;
    }

    RCLCPP_INFO(node->get_logger(), "result received");
    for (auto number : result.response.get()->sequence) {
        RCLCPP_INFO(node->get_logger(), "%"
                PRId64, number);
    }

    rclcpp::shutdown();

    return 0;
}