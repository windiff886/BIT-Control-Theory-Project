#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class OdomToTf : public rclcpp::Node
{
public:
  OdomToTf() : Node("odom_to_tf")
  {
    // 订阅 /odom 话题
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&OdomToTf::odomCallback, this, std::placeholders::_1));
    
    // 创建 tf broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    RCLCPP_INFO(this->get_logger(), "odom_to_tf 节点已启动，正在将 /odom 转换为 tf...");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped t;
    
    // 使用消息的时间戳
    t.header.stamp = msg->header.stamp;
    // 父坐标系设为 odom（不带命名空间前缀）
    t.header.frame_id = "odom";
    // 子坐标系设为 body_link（不带命名空间前缀）
    t.child_frame_id = "body_link";
    
    // 直接使用 Gazebo 发布的位姿（修复后应该是准确的）
    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = msg->pose.pose.position.z;
    t.transform.rotation = msg->pose.pose.orientation;
    
    // 发布 tf
    tf_broadcaster_->sendTransform(t);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomToTf>());
  rclcpp::shutdown();
  return 0;
}
