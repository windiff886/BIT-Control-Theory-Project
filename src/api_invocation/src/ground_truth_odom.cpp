#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class GroundTruthOdom : public rclcpp::Node
{
public:
  GroundTruthOdom() : Node("ground_truth_odom")
  {
    // 订阅 Gazebo ground truth pose
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/ground_truth/pose", 10,
      std::bind(&GroundTruthOdom::poseCallback, this, std::placeholders::_1));
    
    // 发布准确的 odometry（使用 ground truth）
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    
    RCLCPP_INFO(this->get_logger(), "Ground Truth Odometry 节点已启动");
    RCLCPP_INFO(this->get_logger(), "订阅: /ground_truth/pose");
    RCLCPP_INFO(this->get_logger(), "发布: /odom (基于 Gazebo 真实位置，无漂移)");
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // 构造 Odometry 消息
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = this->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "body_link";
    
    // 位置（直接从 Gazebo ground truth）
    odom.pose.pose = msg->pose;
    
    // 速度估计（简单的数值微分）
    if (last_pose_time_.seconds() > 0) {
      double dt = (this->now() - last_pose_time_).seconds();
      if (dt > 0.001 && dt < 0.2) {  // 防止异常时间差
        double dx = msg->pose.position.x - last_x_;
        double dy = msg->pose.position.y - last_y_;
        
        odom.twist.twist.linear.x = std::hypot(dx, dy) / dt;
        
        // 计算角速度
        tf2::Quaternion q_current(
          msg->pose.orientation.x,
          msg->pose.orientation.y,
          msg->pose.orientation.z,
          msg->pose.orientation.w
        );
        tf2::Matrix3x3 m_current(q_current);
        double roll, pitch, yaw_current;
        m_current.getRPY(roll, pitch, yaw_current);
        
        double dyaw = yaw_current - last_yaw_;
        // 归一化角度差
        while (dyaw > M_PI) dyaw -= 2 * M_PI;
        while (dyaw < -M_PI) dyaw += 2 * M_PI;
        
        odom.twist.twist.angular.z = dyaw / dt;
        
        last_yaw_ = yaw_current;
      }
    }
    
    // 保存当前状态
    last_x_ = msg->pose.position.x;
    last_y_ = msg->pose.position.y;
    last_pose_time_ = this->now();
    
    // 发布
    odom_pub_->publish(odom);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  
  double last_x_ = 0.0;
  double last_y_ = 0.0;
  double last_yaw_ = 0.0;
  rclcpp::Time last_pose_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundTruthOdom>());
  rclcpp::shutdown();
  return 0;
}
