//
// Created by zdy on 19-5-22.
//

#include <chrono>
#include <cinttypes>
#include <memory>
#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using AddTwoInts = example_interfaces::srv::AddTwoInts;

int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("minimal_client");
    auto client = node->create_client<AddTwoInts>("add_two_ints", rmw_qos_profile_services_default);

    while (!client->wait_for_service(std::chrono::seconds(1))) {

        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
            return 1;
        }

        RCLCPP_INFO(node->get_logger(), "waiting for service to appear...");
    }


    int n = 50000;
    while (n-- > 0) {

        auto request = std::make_shared<AddTwoInts::Request>();
        request->a = 41;
        request->b = 1;

        auto result_future = client->async_send_request(request);

        if (rclcpp::spin_until_future_complete(node, result_future) != rclcpp::executor::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node->get_logger(), "service call failed :(");
            return 1;
        }

        auto result = result_future.get();
        RCLCPP_INFO(node->get_logger(), "result of %"
                PRId64
                " + %"
                PRId64
                " = %"
                PRId64,
                    request->a, request->b, result->sum);

    }


    rclcpp::shutdown();

    return 0;
}