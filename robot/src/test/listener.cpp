//
// Created by zdy on 19-5-11.
//

//#include <cstdio>
//#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


// Create a Listener class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class Listener : public rclcpp::Node {
public:
    explicit Listener(const std::string &topic_name) : Node("listener") {

        // Create a callback function for when messages are received.
        // Variations of this function also exist using, for example UniquePtr for zero-copy transport.
        auto callback = [this](const std_msgs::msg::String::SharedPtr msg) -> void {
            RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg->data.c_str());
        };

        // Create a subscription to the topic which can be matched with one or more compatible ROS
        // publishers.
        // Note that not all publishers on the same topic with the same type will be compatible:
        // they must have compatible Quality of Service policies.

//        rmw_qos_profile_t qos_profile;
//        qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
//        qos_profile.depth = 7;
//        qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE; //RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT RMW_QOS_POLICY_RELIABILITY_RELIABLE
//        qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

        sub_ = create_subscription<std_msgs::msg::String>(topic_name, callback, rmw_qos_profile_default); //rmw_qos_profile_sensor_data
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char *argv[]) {

    // Force flush of the stdout buffer.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);


    // Initialize any global resources needed by the middleware and the client library.
    // You must call this before using any other part of the ROS system.
    // This should be called once per process.
    rclcpp::init(argc, argv);

    // Parse the command line options.
    auto topic = std::string("chatter");

    // Create a node.
    auto node = std::make_shared<Listener>(topic);

    // spin will block until work comes in, execute work as it becomes available, and keep blocking.
    // It will only be interrupted by Ctrl-C.
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}