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
      "navigation_command", 10, 
      std::bind(&AdvancedExecutor::cmdCallback, this, std::placeholders::_1));
    
    // ✅ 订阅真实的 Odometry 话题
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&AdvancedExecutor::odomCallback, this, std::placeholders::_1));
    
    // 发布速度与机械臂
    pub_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    pub_arm_ = this->create_publisher<std_msgs::msg::String>("arm_command", 10);

    // 控制循环 20Hz (50ms)
    timer_ = this->create_wall_timer(50ms, std::bind(&AdvancedExecutor::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "高级执行节点(使用真实Odom + 防重复)已启动");
  }

private:
  // ✅ Odometry 回调函数
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // 提取位置信息
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    
    // 提取四元数并转换为欧拉角 (Yaw)
    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w
    );
    
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, current_yaw_);
    
    // 提取速度信息
    current_linear_vel_ = msg->twist.twist.linear.x;
    current_angular_vel_ = msg->twist.twist.angular.z;
    
    odom_received_ = true;
  }

  void cmdCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    // ✅ 防止重复任务: 检查是否与上次命令相同
    std::string incoming_command = msg->data;
    
    if (incoming_command == last_command_) {
      RCLCPP_WARN(this->get_logger(), "检测到重复指令，忽略 (若需重复执行，请先发送 'stop' 指令)");
      return;
    }
    
    // ✅ 如果正在执行任务，询问是否清空队列
    if (has_active_task_ || !task_queue_.empty()) {
      RCLCPP_WARN(this->get_logger(), 
        "检测到新指令，但当前还有任务在执行！清空旧任务队列并执行新指令。");
      task_queue_.clear();
      has_active_task_ = false;
      stopRobot();
    }
    
    // 保存此次命令
    last_command_ = incoming_command;

    Json::Value root;
    Json::CharReaderBuilder reader;
    std::string errs;
    std::istringstream stream(msg->data);

    if (Json::parseFromStream(reader, stream, &root, &errs)) {
      if (root.isArray()) {
        int task_count = 0;
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
          task_count++;
        }
        RCLCPP_INFO(this->get_logger(), "收到新任务序列，共 %d 个任务", task_count);
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "JSON 解析失败: %s", errs.c_str());
    }
  }

  void controlLoop()
  {
    // ✅ 检查是否收到 odometry 数据
    if (!odom_received_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                           "等待 odometry 数据...");
      return;
    }

    if (!has_active_task_) {
      if (!task_queue_.empty()) {
        current_task_ = task_queue_.front();
        task_queue_.pop_front();
        has_active_task_ = true;
        task_start_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "开始执行: %s", current_task_.action.c_str());
      } else {
        // ✅ 队列空闲时停车，并清空上次命令记录
        stopRobot();
        if (!last_command_.empty()) {
          RCLCPP_INFO(this->get_logger(), "所有任务执行完毕，等待新指令...");
          last_command_.clear();  // 清空记录，允许下次相同指令
        }
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
      last_command_.clear();  // 清空命令记录
      RCLCPP_INFO(this->get_logger(), "已停止");
    }
  }

  void executeMoveCmd()
  {
    double elapsed = (this->now() - task_start_time_).seconds();
    if (elapsed < current_task_.duration) {
      publishVelocity(current_task_.linear, current_task_.angular);
    } else {
      stopRobot();
      RCLCPP_INFO(this->get_logger(), "✓ move_cmd 完成");
      has_active_task_ = false;
    }
  }

  void executeWait()
  {
    if ((this->now() - task_start_time_).seconds() >= current_task_.duration) {
      RCLCPP_INFO(this->get_logger(), "✓ 等待结束");
      has_active_task_ = false;
    }
  }

  void executeArm()
  {
    std_msgs::msg::String msg;
    msg.data = current_task_.command;
    pub_arm_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "✓ 机械臂: %s", current_task_.command.c_str());
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
        RCLCPP_INFO(this->get_logger(), 
          "导航: 当前(%.2f, %.2f) Yaw:%.2f° | 目标(%.2f, %.2f) Dist:%.2f",
          current_x_, current_y_, current_yaw_ * 180.0 / M_PI, 
          current_task_.x, current_task_.y, distance);
    }

    if (distance < 0.3) { 
      stopRobot();
      RCLCPP_INFO(this->get_logger(), "到达目标 (%.2f, %.2f)", 
                  current_task_.x, current_task_.y);
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
  }

  void stopRobot()
  {
    publishVelocity(0.0, 0.0);
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_command_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_arm_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::deque<Task> task_queue_;
  Task current_task_;
  bool has_active_task_ = false;
  rclcpp::Time task_start_time_;

  // 状态变量 (由 odometry 更新)
  double current_x_ = 0.0;
  double current_y_ = 0.0;
  double current_yaw_ = 0.0;
  
  double current_linear_vel_ = 0.0;
  double current_angular_vel_ = 0.0;
  
  bool odom_received_ = false;
  
  // ✅ 新增: 记录上次命令，防止重复执行
  std::string last_command_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AdvancedExecutor>());
  rclcpp::shutdown();
  return 0;
}