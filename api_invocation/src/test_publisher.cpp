#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

class VoiceCommandPublisher : public rclcpp::Node
{
public:
  VoiceCommandPublisher() : Node("voice_command_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("voice_command", 10);
    timer_ = this->create_wall_timer(
      5s, std::bind(&VoiceCommandPublisher::publishCommand, this));
    
    // 测试指令
    test_commands_ = {
      "朝左前方沿与朝向30度方向前进2m",
      "前进5米然后左转",
      "先右转再左转",
      "去厨房拿一杯水",
      "画一条sinx曲线,等待3s后返回"
    };
    
    RCLCPP_INFO(this->get_logger(), "Voice Command Publisher started");
  }

private:
  void publishCommand()
  {
    auto message = std_msgs::msg::String();
    message.data = test_commands_[command_index_];
    
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    
    command_index_ = (command_index_ + 1) % test_commands_.size();
  }
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::string> test_commands_;
  size_t command_index_ = 0;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoiceCommandPublisher>());
  rclcpp::shutdown();
  return 0;
}
