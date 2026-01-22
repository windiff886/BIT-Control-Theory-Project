#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <json/json.h> // 使用 jsoncpp 库
#include <chrono>
#include <thread>
#include <string>

using namespace std::chrono_literals;

class SimpleExecutor : public rclcpp::Node
{
public:
  SimpleExecutor() : Node("simple_executor")
  {
    // 1. 订阅 LLM 解析后的指令话题
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "navigation_command", 10,
      std::bind(&SimpleExecutor::commandCallback, this, std::placeholders::_1));

    // 2. 创建发布者，控制小车运动 (cmd_vel)
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "执行节点(C++)已启动，等待 navigation_command 指令...");
  }

private:
  void commandCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "收到指令: %s", msg->data.c_str());

    Json::Value root;
    Json::CharReaderBuilder reader;
    std::string errs;
    std::istringstream stream(msg->data);

    // 解析 JSON
    if (Json::parseFromStream(reader, stream, &root, &errs)) {
      try {
        // 提取参数，提供默认值
        double linear_x = root.get("linear", 0.0).asDouble();
        double angular_z = root.get("angular", 0.0).asDouble();
        double duration = root.get("duration", 0.0).asDouble();

        executeMotion(linear_x, angular_z, duration);
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "处理 JSON 数据时出错: %s", e.what());
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "解析 JSON 失败: %s", errs.c_str());
    }
  }

  void executeMotion(double linear, double angular, double duration)
  {
    RCLCPP_INFO(this->get_logger(), "开始执行: 线速度=%.2f, 角速度=%.2f, 时长=%.2f秒", 
                linear, angular, duration);

    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = linear;
    twist.angular.z = angular;

    // 计算结束时间
    auto start_time = std::chrono::steady_clock::now();
    auto duration_ms = std::chrono::milliseconds(static_cast<int>(duration * 1000));
    
    rclcpp::Rate rate(10); // 10Hz

    // 循环发布速度指令
    while (rclcpp::ok()) {
      auto now = std::chrono::steady_clock::now();
      if (now - start_time >= duration_ms) {
        break;
      }
      
      cmd_vel_pub_->publish(twist);
      rate.sleep();
    }

    // 时间到，停车
    stopRobot();
    RCLCPP_INFO(this->get_logger(), "任务完成，已停车。");
  }

  void stopRobot()
  {
    auto stop_twist = geometry_msgs::msg::Twist();
    stop_twist.linear.x = 0.0;
    stop_twist.angular.z = 0.0;
    cmd_vel_pub_->publish(stop_twist);
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleExecutor>());
  rclcpp::shutdown();
  return 0;
}