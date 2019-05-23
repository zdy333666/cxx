//
// Created by zdy on 19-5-20.
//

#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include "opencv2/highgui/highgui.hpp"

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"

#include "./burger.hpp"

/// Convert an OpenCV matrix encoding type to a string format recognized by sensor_msgs::Image.
/**
 * \param[in] mat_type The OpenCV encoding type.
 * \return A string representing the encoding type.
 */
std::string
mat_type2encoding(int mat_type) {
    switch (mat_type) {
        case CV_8UC1:
            return "mono8";
        case CV_8UC3:
            return "bgr8";
        case CV_16SC1:
            return "mono16";
        case CV_8UC4:
            return "rgba8";
        default:
            throw std::runtime_error("Unsupported encoding type");
    }
}

/// Convert an OpenCV matrix (cv::Mat) to a ROS Image message.
/**
 * \param[in] frame The OpenCV matrix/image to convert.
 * \param[in] frame_id ID for the ROS message.
 * \param[out] Allocated shared pointer for the ROS Image message.
 */
void convert_frame_to_message(const cv::Mat &frame, size_t frame_id, sensor_msgs::msg::Image &msg) {
    // copy cv information into ros message
    msg.height = frame.rows;
    msg.width = frame.cols;
    msg.encoding = mat_type2encoding(frame.type());
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
    size_t size = frame.step * frame.rows;
    msg.data.resize(size);
    memcpy(&msg.data[0], frame.data, size);
    msg.header.frame_id = std::to_string(frame_id);
}

int main(int argc, char *argv[]) {

    // Pass command line arguments to rclcpp.
    rclcpp::init(argc, argv);

    // Initialize default demo parameters
    bool show_camera = false;
    double freq = 30.0;

    size_t width = 320;
    size_t height = 240;

    bool burger_mode = false;
    std::string topic("image");

    // Force flush of the stdout buffer.
    // This ensures a correct sync of all prints
    // even when executed simultaneously within a launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Initialize a ROS 2 node to publish images read from the OpenCV interface to the camera.
    auto node = rclcpp::Node::make_shared("cam2image");
    rclcpp::Logger node_logger = node->get_logger();

    RCLCPP_INFO(node_logger, "Publishing data on topic '%s'", topic.c_str());

    // Create the image publisher with our custom QoS profile.
    rmw_qos_profile_t qos_profile;
    qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    qos_profile.depth = 1;
    qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

    auto pub = node->create_publisher<sensor_msgs::msg::Image>(topic, qos_profile);

    // is_flipped will cause the incoming camera image message to flip about the y-axis.
    bool is_flipped = false;

    // Subscribe to a message that will toggle flipping or not flipping, and manage the state in a
    // callback.
    auto callback =
            [&is_flipped, &node_logger](const std_msgs::msg::Bool::SharedPtr msg) -> void {
                is_flipped = msg->data;
                RCLCPP_INFO(node_logger, "Set flip mode to: %s", is_flipped ? "on" : "off");
            };

    // Set the QoS profile for the subscription to the flip message.
    auto sub = node->create_subscription<std_msgs::msg::Bool>("flip_image", callback, 1);

    // Set a loop rate for our main event loop.
    rclcpp::WallRate loop_rate(freq);

    cv::VideoCapture cap;
    burger::Burger burger_cap;
    if (!burger_mode) {
        // Initialize OpenCV video capture stream.
        // Always open device 0.
        cap.open(0);

        // Set the width and height based on command line arguments.
        cap.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(width));
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(height));
        if (!cap.isOpened()) {
            RCLCPP_ERROR(node_logger, "Could not open video stream");
            return 1;
        }
    }

    // Initialize OpenCV image matrices.
    cv::Mat frame;
    cv::Mat flipped_frame;

    size_t i = 1;

    // Our main event loop will spin until the user presses CTRL-C to exit.
    while (rclcpp::ok()) {
        // Initialize a shared pointer to an Image message.
        auto msg = std::make_shared<sensor_msgs::msg::Image>();
        msg->is_bigendian = false;

        // Get the frame from the video capture.
        if (burger_mode) {
            frame = burger_cap.render_burger(width, height);
        } else {
            cap >> frame;
        }

        // Check if the frame was grabbed correctly
        if (!frame.empty()) {

            // Convert to a ROS image
            if (!is_flipped) {
                convert_frame_to_message(frame, i, *msg);
            } else {
                // Flip the frame if needed
                cv::flip(frame, flipped_frame, 1);
                convert_frame_to_message(flipped_frame, i, *msg);
            }

            if (show_camera) {
                cv::Mat cvframe = frame;
                // Show the image in a window called "cam2image".
                cv::imshow("cam2image", cvframe);
                // Draw the image to the screen and wait 1 millisecond.
                cv::waitKey(1);
            }

            // Publish the image message and increment the frame_id.
            RCLCPP_INFO(node_logger, "Publishing image #%zd", i);
            pub->publish(std::move(msg));

            ++i;
        }

        // Do some work in rclcpp and wait for more to come in.
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}