#include "cv_bridge_4/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/highgui.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <functional>
#include <memory>
#include <opencv2/opencv.hpp>

class ImageSubscriberNode : public rclcpp::Node {
public:
  ImageSubscriberNode() : Node("img_subscriber") {
    subscriber_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "/fast_img",
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local(),
        std::bind(&ImageSubscriberNode::image_callback, this, std::placeholders::_1));
  }

private:
  void image_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
    rclcpp::Time now = this->now();
    rclcpp::Duration delay = now - msg->header.stamp;
    RCLCPP_INFO(this->get_logger(), "delay = %f s", delay.seconds());
    // cv::Mat image = cv::imdecode(msg->data, cv::IMREAD_UNCHANGED);
    // cv::imshow("received", image);
    // cv::waitKey(1);
  }

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscriber_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageSubscriberNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
