#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>
#include <string>
#include <thread>

class VoiceInputNode : public rclcpp::Node
{
public:
  VoiceInputNode() : Node("voice_input_node")
  {
    // 创建发布者 - 发布自然语言指令到 voice_command 话题
    voice_command_pub_ = this->create_publisher<std_msgs::msg::String>("voice_command", 10);
    
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════════════╗\n";
    std::cout << "║         自然语言导航命令输入终端               ║\n";
    std::cout << "╠════════════════════════════════════════════════╣\n";
    std::cout << "║  支持的指令类型:                               ║\n";
    std::cout << "║   1. 坐标导航: \"去坐标(2,2)\" | \"走到(1.5,3.0)\" ║\n";
    std::cout << "║   2. 语义导航: \"去茶几前面\" | \"走到两个球中间\" ║\n";
    std::cout << "║   3. 基础移动: \"向前走3秒\" | \"左转90度\"        ║\n";
    std::cout << "║   4. 复合任务: \"去坐标(2,2)，然后抬起机械臂\"   ║\n";
    std::cout << "║   5. 曲线绘制: \"画一条sinx曲线，等待3s后返回\"  ║\n";
    std::cout << "║   6. 相对移动: \"朝左前方30度方向前进2m\"       ║\n";
    std::cout << "║   7. 控制指令: \"停止\" | \"等待5秒\"             ║\n";
    std::cout << "╠════════════════════════════════════════════════╣\n";
    std::cout << "║  输入 'quit' 退出                              ║\n";
    std::cout << "╚════════════════════════════════════════════════╝\n";
    std::cout << std::endl;
    
    // 启动输入线程
    input_thread_ = std::thread(&VoiceInputNode::inputLoop, this);
  }
  
  ~VoiceInputNode()
  {
    running_ = false;
    if (input_thread_.joinable()) {
      input_thread_.join();
    }
  }

private:
  void inputLoop()
  {
    std::string input;
    
    while (running_ && rclcpp::ok()) {
      std::cout << "\n🎤 请输入指令> " << std::flush;
      
      if (!std::getline(std::cin, input)) {
        break;
      }
      
      // 去除首尾空白
      input.erase(0, input.find_first_not_of(" \t\n\r"));
      input.erase(input.find_last_not_of(" \t\n\r") + 1);
      
      if (input.empty()) {
        continue;
      }
      
      // 检查退出命令
      if (input == "quit" || input == "exit" || input == "q") {
        RCLCPP_INFO(this->get_logger(), "正在退出...");
        running_ = false;
        rclcpp::shutdown();
        break;
      }
      
      // 发布指令
      auto msg = std_msgs::msg::String();
      msg.data = input;
      voice_command_pub_->publish(msg);
      
      RCLCPP_INFO(this->get_logger(), "✓ 已发送指令: \"%s\"", input.c_str());
      RCLCPP_INFO(this->get_logger(), "  等待LLM解析中...");
    }
  }
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr voice_command_pub_;
  std::thread input_thread_;
  std::atomic<bool> running_{true};
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VoiceInputNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
