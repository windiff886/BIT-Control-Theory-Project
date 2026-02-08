#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <string>

using namespace std::chrono_literals;

class ImageViewer : public rclcpp::Node
{
public:
  ImageViewer() : Node("image_viewer")
  {
    // 声明参数
    this->declare_parameter("image_topic", "/camera/image_raw");
    this->declare_parameter("window_name", "🤖 Robot Camera - AI Vision Input");
    this->declare_parameter("save_images", false);
    this->declare_parameter("save_path", "/tmp/robot_images");
    this->declare_parameter("show_info", true);
    
    image_topic_ = this->get_parameter("image_topic").as_string();
    window_name_ = this->get_parameter("window_name").as_string();
    save_images_ = this->get_parameter("save_images").as_bool();
    save_path_ = this->get_parameter("save_path").as_string();
    show_info_ = this->get_parameter("show_info").as_bool();
    
    // 订阅相机图像话题
    sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_, 10,
      std::bind(&ImageViewer::imageCallback, this, std::placeholders::_1));
    
    // 创建显示窗口
    cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
    
    // 用于处理 OpenCV GUI 事件的定时器
    timer_ = this->create_wall_timer(30ms, std::bind(&ImageViewer::displayLoop, this));
    
    RCLCPP_INFO(this->get_logger(), "===================================");
    RCLCPP_INFO(this->get_logger(), "📷 图像查看器已启动");
    RCLCPP_INFO(this->get_logger(), "订阅话题: %s", image_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "窗口名称: %s", window_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "-----------------------------------");
    RCLCPP_INFO(this->get_logger(), "快捷键:");
    RCLCPP_INFO(this->get_logger(), "  q/ESC - 退出");
    RCLCPP_INFO(this->get_logger(), "  s     - 保存当前帧");
    RCLCPP_INFO(this->get_logger(), "  i     - 切换信息显示");
    RCLCPP_INFO(this->get_logger(), "===================================");
    
    if (save_images_) {
      // 创建保存目录
      std::string cmd = "mkdir -p " + save_path_;
      system(cmd.c_str());
      RCLCPP_INFO(this->get_logger(), "自动保存图像到: %s", save_path_.c_str());
    }
  }
  
  ~ImageViewer()
  {
    cv::destroyAllWindows();
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    
    try {
      // 将 ROS Image 转换为 OpenCV Mat
      if (msg->encoding == "rgb8") {
        latest_image_ = cv::Mat(msg->height, msg->width, CV_8UC3, 
                               const_cast<unsigned char*>(msg->data.data())).clone();
        cv::cvtColor(latest_image_, latest_image_, cv::COLOR_RGB2BGR);
      } else if (msg->encoding == "bgr8") {
        latest_image_ = cv::Mat(msg->height, msg->width, CV_8UC3, 
                               const_cast<unsigned char*>(msg->data.data())).clone();
      } else if (msg->encoding == "mono8") {
        latest_image_ = cv::Mat(msg->height, msg->width, CV_8UC1, 
                               const_cast<unsigned char*>(msg->data.data())).clone();
        cv::cvtColor(latest_image_, latest_image_, cv::COLOR_GRAY2BGR);
      } else if (msg->encoding == "rgba8") {
        cv::Mat temp(msg->height, msg->width, CV_8UC4, 
                    const_cast<unsigned char*>(msg->data.data()));
        cv::cvtColor(temp, latest_image_, cv::COLOR_RGBA2BGR);
        latest_image_ = latest_image_.clone();
      } else if (msg->encoding == "bgra8") {
        cv::Mat temp(msg->height, msg->width, CV_8UC4, 
                    const_cast<unsigned char*>(msg->data.data()));
        cv::cvtColor(temp, latest_image_, cv::COLOR_BGRA2BGR);
        latest_image_ = latest_image_.clone();
      } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "不支持的图像编码格式: %s", msg->encoding.c_str());
        return;
      }
      
      image_received_ = true;
      frame_count_++;
      
      // 自动保存模式
      if (save_images_ && frame_count_ % 30 == 0) {  // 每30帧保存一张
        saveCurrentFrame();
      }
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "图像处理错误: %s", e.what());
    }
  }
  
  void displayLoop()
  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    
    if (!image_received_ || latest_image_.empty()) {
      return;
    }
    
    // 创建显示图像
    cv::Mat display_image = latest_image_.clone();
    
    // 添加信息文字（可切换）
    if (show_info_) {
      // 半透明背景
      cv::Mat overlay = display_image.clone();
      cv::rectangle(overlay, cv::Point(5, 5), cv::Point(200, 80), 
                    cv::Scalar(0, 0, 0), -1);
      cv::addWeighted(overlay, 0.5, display_image, 0.5, 0, display_image);
      
      std::string info = "Frame: " + std::to_string(frame_count_);
      cv::putText(display_image, info, cv::Point(10, 25), 
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
      
      std::string size_info = "Size: " + std::to_string(display_image.cols) + "x" + std::to_string(display_image.rows);
      cv::putText(display_image, size_info, cv::Point(10, 45), 
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
      
      // 计算帧率
      auto now = std::chrono::steady_clock::now();
      double elapsed = std::chrono::duration<double>(now - last_fps_time_).count();
      if (elapsed >= 1.0) {
        current_fps_ = (frame_count_ - last_fps_frame_) / elapsed;
        last_fps_time_ = now;
        last_fps_frame_ = frame_count_;
      }
      std::string fps_info = "FPS: " + std::to_string(static_cast<int>(current_fps_));
      cv::putText(display_image, fps_info, cv::Point(10, 65), 
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
      
      // 底部提示
      cv::putText(display_image, "AI Vision Input | Press 'i' to hide info", 
                  cv::Point(10, display_image.rows - 10), 
                  cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 0), 1);
    }
    
    // 显示图像
    cv::imshow(window_name_, display_image);
    
    // 处理键盘事件
    int key = cv::waitKey(1);
    if (key == 'q' || key == 'Q' || key == 27) {  // q 或 ESC 退出
      RCLCPP_INFO(this->get_logger(), "用户请求退出...");
      rclcpp::shutdown();
    } else if (key == 's' || key == 'S') {  // s 保存当前帧
      saveCurrentFrame();
    } else if (key == 'i' || key == 'I') {  // i 切换信息显示
      show_info_ = !show_info_;
      RCLCPP_INFO(this->get_logger(), "信息显示: %s", show_info_ ? "开启" : "关闭");
    }
  }
  
  void saveCurrentFrame()
  {
    if (latest_image_.empty()) return;
    
    auto now = std::chrono::system_clock::now();
    auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
      now.time_since_epoch()).count();
    
    std::string filename = save_path_ + "/frame_" + std::to_string(timestamp) + ".jpg";
    
    if (cv::imwrite(filename, latest_image_)) {
      RCLCPP_INFO(this->get_logger(), "✅ 图像已保存: %s", filename.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "❌ 图像保存失败: %s", filename.c_str());
    }
  }
  
  // ROS 订阅和定时器
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  // 参数
  std::string image_topic_;
  std::string window_name_;
  bool save_images_;
  std::string save_path_;
  bool show_info_;
  
  // 图像数据
  cv::Mat latest_image_;
  bool image_received_ = false;
  uint64_t frame_count_ = 0;
  std::mutex image_mutex_;
  
  // FPS 计算
  std::chrono::steady_clock::time_point last_fps_time_ = std::chrono::steady_clock::now();
  uint64_t last_fps_frame_ = 0;
  double current_fps_ = 0.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  try {
    rclcpp::spin(std::make_shared<ImageViewer>());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("image_viewer"), "异常: %s", e.what());
  }
  
  rclcpp::shutdown();
  return 0;
}
