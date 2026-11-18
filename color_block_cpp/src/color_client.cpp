#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "color_block_cpp/srv/set_target_color.hpp"

using SetTargetColor = color_block_cpp::srv::SetTargetColor;

class ColorClient : public rclcpp::Node
{
public:
  ColorClient() : Node("color_client")
  {
    client_ = this->create_client<SetTargetColor>("/set_target_color");
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(this->get_logger(), "等待 /set_target_color 服务...");
    }
  }

  bool send(const std::string & color)
  {
    auto request = std::make_shared<SetTargetColor::Request>();
    request->color = color;

    auto future = client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(
          this->get_node_base_interface(), future, std::chrono::seconds(5)) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      std::cout << "服务调用超时" << std::endl;
      return false;
    }

    auto response = future.get();
    std::cout << (response->success ? "[成功] " : "[失败] ")
              << response->message << std::endl;
    return response->success;
  }

private:
  rclcpp::Client<SetTargetColor>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ColorClient>();

  std::cout << "输入要识别的颜色（red / green / blue），输入 q 退出：" << std::endl;
  std::string line;
  while (rclcpp::ok()) {
    std::cout << "> ";
    if (!std::getline(std::cin, line)) {
      break;
    }
    if (line == "q" || line == "Q") {
      break;
    }
    if (!line.empty()) {
      node->send(line);
    }
  }

  rclcpp::shutdown();
  return 0;
}
