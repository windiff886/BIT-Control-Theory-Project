#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <json/json.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <deque>
#include <string>

using namespace std::chrono_literals;

struct Task {
  std::string action;
  double linear = 0.0;
  double angular = 0.0;
  double duration = 0.0;
  double x = 0.0;
  double y = 0.0;
  std::string command;
};

class AdvancedExecutor : public rclcpp::Node
{
public:
  AdvancedExecutor() : Node("advanced_executor")
  {
    // 订阅指令
    sub_command_ = this->create_subscription<std_msgs::msg::String>(
      "navigation_command", 10, std::bind(&AdvancedExecutor::cmdCallback, this, std::placeholders::_1));
    
    // 【弃用】既然 Gazebo Odom 是坏的，我们不再订阅它，改用自算
    // sub_odom_ = ... 
    
    // 发布速度与机械臂
    pub_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    pub_arm_ = this->create_publisher<std_msgs::msg::String>("arm_command", 10);

    // 控制循环 20Hz (50ms)
    timer_ = this->create_wall_timer(50ms, std::bind(&AdvancedExecutor::controlLoop, this));

    last_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "高级执行节点(自算Odom版)已启动");
  }

private:
  void cmdCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    Json::Value root;
    Json::CharReaderBuilder reader;
    std::string errs;
    std::istringstream stream(msg->data);

    if (Json::parseFromStream(reader, stream, &root, &errs)) {
      if (root.isArray()) {
        for (const auto& item : root) {
          Task task;
          task.action = item.get("action", "").asString();
          task.linear = item.get("linear", 0.0).asDouble();
          task.angular = item.get("angular", 0.0).asDouble();
          task.duration = item.get("duration", 0.0).asDouble();
          task.x = item.get("x", 0.0).asDouble();
          task.y = item.get("y", 0.0).asDouble();
          task.command = item.get("command", "").asString();
          task_queue_.push_back(task);
        }
        RCLCPP_INFO(this->get_logger(), "收到任务序列，当前待执行: %ld个", task_queue_.size());
      }
    }
  }

  void controlLoop()
  {
    // === 核心修改：手动计算里程计 (Dead Reckoning) ===
    rclcpp::Time now = this->now();
    double dt = (now - last_time_).seconds();
    if (dt > 1.0) dt = 0.05; // 防止首帧跳变
    last_time_ = now;

    // 运动学积分：新位置 = 旧位置 + 速度 * 时间
    // 假设是差速/阿克曼简化模型
    current_yaw_ += last_angular_vel_ * dt;
    // 归一化 Yaw
    while (current_yaw_ > M_PI) current_yaw_ -= 2 * M_PI;
    while (current_yaw_ < -M_PI) current_yaw_ += 2 * M_PI;

    current_x_ += last_linear_vel_ * std::cos(current_yaw_) * dt;
    current_y_ += last_linear_vel_ * std::sin(current_yaw_) * dt;

    // ===================================================

    if (!has_active_task_) {
      if (!task_queue_.empty()) {
        current_task_ = task_queue_.front();
        task_queue_.pop_front();
        has_active_task_ = true;
        task_start_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "执行: %s", current_task_.action.c_str());
      } else {
        // 空闲时要停车，并重置记录的速度
        stopRobot();
        return; 
      }
    }

    if (current_task_.action == "move_cmd") executeMoveCmd();
    else if (current_task_.action == "move_to") executeMoveTo();
    else if (current_task_.action == "control_arm") executeArm();
    else if (current_task_.action == "wait") executeWait();
    else if (current_task_.action == "stop") {
      stopRobot();
      task_queue_.clear();
      has_active_task_ = false;
    }
  }

  void executeMoveCmd()
  {
    double elapsed = (this->now() - task_start_time_).seconds();
    if (elapsed < current_task_.duration) {
      publishVelocity(current_task_.linear, current_task_.angular);
    } else {
      stopRobot();
      has_active_task_ = false;
    }
  }

  void executeWait()
  {
    if ((this->now() - task_start_time_).seconds() >= current_task_.duration) {
      RCLCPP_INFO(this->get_logger(), "等待结束");
      has_active_task_ = false;
    }
  }

  void executeArm()
  {
    std_msgs::msg::String msg;
    msg.data = current_task_.command;
    pub_arm_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "机械臂: %s", current_task_.command.c_str());
    rclcpp::sleep_for(1s);
    has_active_task_ = false;
  }

  void executeMoveTo()
  {
    double dx = current_task_.x - current_x_;
    double dy = current_task_.y - current_y_;
    double distance = std::sqrt(dx*dx + dy*dy);

    // 调试日志 (每0.5秒打印一次)
    static int print_count = 0;
    if (print_count++ % 10 == 0) {
        RCLCPP_INFO(this->get_logger(), "自算导航: 当前(%.2f, %.2f) Yaw:%.2f | 目标(%.2f, %.2f) Dist:%.2f",
            current_x_, current_y_, current_yaw_, current_task_.x, current_task_.y, distance);
    }

    if (distance < 0.3) { 
      stopRobot();
      RCLCPP_INFO(this->get_logger(), "到达目标 (%.2f, %.2f)", current_task_.x, current_task_.y);
      has_active_task_ = false;
      return;
    }

    double target_angle = std::atan2(dy, dx);
    double angle_diff = target_angle - current_yaw_;

    while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

    // 速度计算
    double v = 0.0;
    double w = 0.0;

    // 转向逻辑
    w = 0.8 * angle_diff;
    if (w > 0.8) w = 0.8;
    if (w < -0.8) w = -0.8;

    // 前进逻辑 (阿克曼必须动起来才能转)
    if (std::abs(angle_diff) > 0.5) v = 0.3; // 偏差大，慢速转弯
    else v = 0.8; // 偏差小，快速前进

    publishVelocity(v, w);
  }

  void publishVelocity(double linear, double angular)
  {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = linear;
    twist.angular.z = angular;
    pub_vel_->publish(twist);
    
    // 关键：记录刚才发出的速度，用于下一次循环计算位置
    last_linear_vel_ = linear;
    last_angular_vel_ = angular;
  }

  void stopRobot()
  {
    publishVelocity(0.0, 0.0);
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_command_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_arm_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::deque<Task> task_queue_;
  Task current_task_;
  bool has_active_task_ = false;
  rclcpp::Time task_start_time_;

  // 状态变量
  double current_x_ = 0.0;
  double current_y_ = 0.0;
  double current_yaw_ = 0.0;
  
  // 积分变量
  double last_linear_vel_ = 0.0;
  double last_angular_vel_ = 0.0;
  rclcpp::Time last_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AdvancedExecutor>());
  rclcpp::shutdown();
  return 0;
}