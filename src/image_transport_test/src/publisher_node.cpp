#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cv_bridge_4/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/imgcodecs.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/detail/compressed_image__struct.hpp"
#include "std_msgs/msg/detail/header__struct.hpp"
#include <memory>
#include <string>

sensor_msgs::msg::CompressedImage
mat_to_msg(const cv::Mat &mat, std::shared_ptr<rclcpp::Node> &node,
           const std::string &format = "jpeg") {
  sensor_msgs::msg::CompressedImage msg;

  std::vector<uchar> buffer;
  cv::imencode("." + format, mat, buffer);
  msg.data = buffer;
  msg.format = format;
  return msg;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("img_publisher");

  auto publisher = node->create_publisher<sensor_msgs::msg::CompressedImage>(
      "/fast_img", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local());
  rclcpp::Rate rate(200);

  // path
  std::string package_path =
      ament_index_cpp::get_package_share_directory("image_transport_test");
  std::string img_path =
      "/media/zhewen/d1/program/robomaster/img-transport-ros2-test/src/"
      "image_transport_test/assets/test.jpg";

  cv::Mat img = cv::imread(img_path, cv::IMREAD_COLOR);
  while (rclcpp::ok()) {
    auto msg = mat_to_msg(img, node);
    msg.header.stamp = node->now();
    publisher->publish(msg);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
