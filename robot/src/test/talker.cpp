//
// Created by zdy on 19-5-10.
//

//#include <chrono>
//#include <cstdio>
//#include <memory>
#include <string>
//#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;


// Create a Talker class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class Talker : public rclcpp::Node {
public:
    explicit Talker(const std::string &topic_name) : Node("talker") {

        // Create a function for when messages are to be sent.
        auto publish_message = [this]() -> void {
            msg_ = std::make_shared<std_msgs::msg::String>();
            msg_->data = "Hello World: " + std::to_string(count_++);
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg_->data.c_str());

            // Put the message into a queue to be processed by the middleware.
            // This call is non-blocking.
            pub_->publish(std::move(msg_));
        };


        // Create a publisher with a custom Quality of Service profile.

//        rmw_qos_profile_t qos_profile;
//        qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
//        qos_profile.depth = 7;
//        qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
//        qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

        pub_ = this->create_publisher<std_msgs::msg::String>(topic_name, rmw_qos_profile_default); //rmw_qos_profile_sensor_data

        // Use a timer to schedule periodic message publishing.
        timer_ = this->create_wall_timer(1ms, publish_message);
    }

private:
    size_t count_ = 1;
    std::shared_ptr <std_msgs::msg::String> msg_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {

    // Force flush of the stdout buffer.
    // This ensures a correct sync of all prints
    // even when executed simultaneously within the launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Initialize any global resources needed by the middleware and the client library.
    // You must call this before using any other part of the ROS system.
    // This should be called once per process.
    rclcpp::init(argc, argv);

    // Parse the command line options.
    auto topic = std::string("chatter");

    // Create a node.
    auto node = std::make_shared<Talker>(topic);

    // spin will block until work comes in, execute work as it becomes available, and keep blocking.
    // It will only be interrupted by Ctrl-C.
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}