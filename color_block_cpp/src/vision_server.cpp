#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

#include "foxglove_msgs/msg/image_annotations.hpp"
#include "foxglove_msgs/msg/points_annotation.hpp"
#include "foxglove_msgs/msg/point2.hpp"
#include "foxglove_msgs/msg/color.hpp"

#include "color_block_cpp/srv/set_target_color.hpp"

using SetTargetColor = color_block_cpp::srv::SetTargetColor;
using std::placeholders::_1;
using std::placeholders::_2;

struct HsvRange {
  int h_low, s_low, v_low;
  int h_high, s_high, v_high;
};

class VisionServer : public rclcpp::Node
{
public:
  VisionServer() : Node("vision_server")
  {
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
    annot_pub_ = this->create_publisher<foxglove_msgs::msg::ImageAnnotations>(
      "/detections/annotations", 10);

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10,
      std::bind(&VisionServer::image_callback, this, _1));

    srv_ = this->create_service<SetTargetColor>(
      "/set_target_color",
      std::bind(&VisionServer::set_color_callback, this, _1, _2));

    ranges_["red"] = {
      {0, 80, 60, 10, 255, 255},
      {156, 80, 60, 180, 255, 255}
    };
    ranges_["green"] = {
      {35, 80, 60, 85, 255, 255}
    };
    ranges_["blue"] = {
      {95, 80, 60, 125, 255, 255}
    };

    target_color_ = "red";
    RCLCPP_INFO(this->get_logger(), "视觉服务端已启动，当前目标颜色: red");
  }

private:
  void set_color_callback(
    const std::shared_ptr<SetTargetColor::Request> request,
    std::shared_ptr<SetTargetColor::Response> response)
  {
    std::string color = request->color;
    std::transform(color.begin(), color.end(), color.begin(), ::tolower);

    if (ranges_.find(color) == ranges_.end()) {
      response->success = false;
      response->message = "不支持的颜色: " + color + "，可用: red, green, blue";
      return;
    }

    target_color_ = color;
    response->success = true;
    response->message = "目标颜色已设置为 " + color;
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    image_pub_->publish(*msg);

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge 转换失败: %s", e.what());
      return;
    }

    cv::Mat frame = cv_ptr->image;
    cv::Mat blurred, hsv, mask;
    cv::GaussianBlur(frame, blurred, cv::Size(5, 5), 0);
    cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

    mask = build_mask(hsv);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    foxglove_msgs::msg::ImageAnnotations annotations;
    annotations.timestamp = msg->header.stamp;

    for (const auto & cnt : contours) {
      double area = cv::contourArea(cnt);
      if (area < 500.0) {
        continue;
      }

      cv::Rect rect = cv::boundingRect(cnt);
      annotations.points.push_back(make_rect_annotation(rect, msg->header.stamp));
    }

    annot_pub_->publish(annotations);
  }

  cv::Mat build_mask(const cv::Mat & hsv)
  {
    cv::Mat mask = cv::Mat::zeros(hsv.size(), CV_8UC1);
    for (const auto & range : ranges_.at(target_color_)) {
      cv::Mat m;
      cv::inRange(
        hsv,
        cv::Scalar(range.h_low, range.s_low, range.v_low),
        cv::Scalar(range.h_high, range.s_high, range.v_high),
        m);
      cv::bitwise_or(mask, m, mask);
    }
    return mask;
  }

  foxglove_msgs::msg::PointsAnnotation make_rect_annotation(
    const cv::Rect & rect, const builtin_interfaces::msg::Time & stamp)
  {
    foxglove_msgs::msg::PointsAnnotation ann;
    ann.timestamp = stamp;
    ann.type = foxglove_msgs::msg::PointsAnnotation::LINE_LOOP;
    ann.thickness = 2.0;

    foxglove_msgs::msg::Color outline;
    outline.r = 0.0f;
    outline.g = 1.0f;
    outline.b = 0.0f;
    outline.a = 1.0f;
    ann.outline_color = outline;

    auto add_point = [&](double x, double y) {
      foxglove_msgs::msg::Point2 p;
      p.x = x;
      p.y = y;
      ann.points.push_back(p);
    };

    add_point(rect.x, rect.y);
    add_point(rect.x + rect.width, rect.y);
    add_point(rect.x + rect.width, rect.y + rect.height);
    add_point(rect.x, rect.y + rect.height);
    add_point(rect.x, rect.y);

    return ann;
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<foxglove_msgs::msg::ImageAnnotations>::SharedPtr annot_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Service<SetTargetColor>::SharedPtr srv_;

  std::string target_color_;
  std::map<std::string, std::vector<HsvRange>> ranges_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
