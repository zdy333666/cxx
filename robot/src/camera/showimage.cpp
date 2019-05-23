//
// Created by zdy on 19-5-20.
//

#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"

/// Convert a sensor_msgs::Image encoding type (stored as a string) to an OpenCV encoding type.
/**
 * \param[in] encoding A string representing the encoding type.
 * \return The OpenCV encoding type.
 */
int encoding2mat_type(const std::string &encoding) {
    if (encoding == "mono8") {
        return CV_8UC1;
    } else if (encoding == "bgr8") {
        return CV_8UC3;
    } else if (encoding == "mono16") {
        return CV_16SC1;
    } else if (encoding == "rgba8") {
        return CV_8UC4;
    } else if (encoding == "bgra8") {
        return CV_8UC4;
    } else if (encoding == "32FC1") {
        return CV_32FC1;
    } else if (encoding == "rgb8") {
        return CV_8UC3;
    } else {
        throw std::runtime_error("Unsupported encoding type");
    }
}

/// Convert the ROS Image message to an OpenCV matrix and display it to the user.
// \param[in] msg The image message to show.
void show_image(const sensor_msgs::msg::Image::SharedPtr msg, bool show_camera, rclcpp::Logger logger) {

    RCLCPP_INFO(logger, "Received image #%s", msg->header.frame_id.c_str());
    std::cerr << "Received image #" << msg->header.frame_id.c_str() << std::endl;

    if (show_camera) {
        // Convert to an OpenCV matrix by assigning the data.
        cv::Mat frame(
                msg->height, msg->width, encoding2mat_type(msg->encoding),
                const_cast<unsigned char *>(msg->data.data()), msg->step);

        if (msg->encoding == "rgb8") {
            cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
        }

        cv::Mat cvframe = frame;

        // Show the image in a window called "showimage".
        cv::imshow("showimage", cvframe);

        // Draw the screen and wait for 1 millisecond.
        cv::waitKey(1);
    }
}

int main(int argc, char *argv[]) {

    // Pass command line arguments to rclcpp.
    rclcpp::init(argc, argv);
    std::cerr << "Right after init" << std::endl;

    bool show_camera = true;
    std::string topic("image");

    // Force flush of the stdout buffer.
    // This ensures a correct sync of all prints
    // even when executed simultaneously within a launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);


    if (show_camera) {
        std::cerr << "Creating window" << std::endl;

        // Initialize an OpenCV named window called "showimage".
        cv::namedWindow("showimage", cv::WINDOW_AUTOSIZE);
        cv::waitKey(1);

        std::cerr << "After creating window" << std::endl;
    }

    // Initialize a ROS node.
    auto node = rclcpp::Node::make_shared("showimage");
    std::cerr << "AFter creating node" << std::endl;

    std::cerr << "Right before defining callback" << std::endl;
    auto callback = [show_camera, &node](const sensor_msgs::msg::Image::SharedPtr msg) {
        show_image(msg, show_camera, node->get_logger());
    };

    std::cerr << "Subscribing to topic '" << topic << "'" << std::endl;
    RCLCPP_INFO(node->get_logger(), "Subscribing to topic '%s'", topic.c_str());

    // Initialize a subscriber that will receive the ROS Image message to be displayed.
    auto sub = node->create_subscription<sensor_msgs::msg::Image>(topic, callback, 1);

    std::cerr << "Spinning" << std::endl;
    rclcpp::spin(node);

    std::cerr << "Shutdown" << std::endl;
    rclcpp::shutdown();

    return 0;
}