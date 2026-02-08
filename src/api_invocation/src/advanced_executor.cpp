#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <json/json.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <curl/curl.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <deque>
#include <string>
#include <vector>
#include <sstream>
#include <map>
#include <future>
#include <atomic>
#include <algorithm>
#include <cctype>

// Base64 编码表
static const std::string base64_chars = 
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789+/";

using namespace std::chrono_literals;

struct Task {
  std::string action;
  double linear = 0.0;
  double angular = 0.0;
  double duration = 0.0;
  double x = 0.0;
  double y = 0.0;
  std::string command;
  // draw_curve 专用字段
  std::string curve_type;
  double amplitude = 1.0;
  double wavelength = 2.0;
  // 语义导航专用字段
  std::string target;       // 目标名称，如 "refrigerator", "bed" 等
  double confidence = 0.80; // 置信度阈值
};

// 视觉识别结果
struct VisionResult {
  double confidence = 0.0;
  bool valid = false;
};

class AdvancedExecutor : public rclcpp::Node
{
public:
  AdvancedExecutor() : Node("advanced_executor")
  {
    // 声明视觉API参数
    this->declare_parameter("vision_api_key", "");
    this->declare_parameter("vision_api_url", "https://api.zhizengzeng.com/chat/completions");
    this->declare_parameter("vision_model", "gpt-4.1");
    this->declare_parameter("vision_system_prompt", 
      "你是一个室内物体识别专家。用户会给你一张图片和一个目标物体名称。"
      "请判断图片中是否包含该目标物体，并返回一个0到1之间的置信度数字。"
      "只输出一个数字，不要有任何其他文字。例如：0.85");
    this->declare_parameter("vision_user_prompt", "请判断这张图片中是否有: {target}。返回置信度(0-1)。");
    
    vision_api_key_ = this->get_parameter("vision_api_key").as_string();
    vision_api_url_ = this->get_parameter("vision_api_url").as_string();
    vision_model_ = this->get_parameter("vision_model").as_string();
    vision_system_prompt_ = this->get_parameter("vision_system_prompt").as_string();
    vision_user_prompt_ = this->get_parameter("vision_user_prompt").as_string();
    
    // 初始化 curl
    curl_global_init(CURL_GLOBAL_DEFAULT);
    
    // 订阅指令
    sub_command_ = this->create_subscription<std_msgs::msg::String>(
      "navigation_command", 10, 
      std::bind(&AdvancedExecutor::cmdCallback, this, std::placeholders::_1));
    
    // 订阅 Odometry
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&AdvancedExecutor::odomCallback, this, std::placeholders::_1));
    
    // 订阅激光雷达用于避障
    sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&AdvancedExecutor::scanCallback, this, std::placeholders::_1));
    
    // 订阅相机图像用于视觉检查
    sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10,
      std::bind(&AdvancedExecutor::imageCallback, this, std::placeholders::_1));
    
    // 发布速度与机械臂
    pub_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    pub_arm_ = this->create_publisher<std_msgs::msg::String>("arm_command", 10);
    
    // 发布轨迹用于 RViz 可视化
    pub_path_ = this->create_publisher<nav_msgs::msg::Path>("/trajectory", 10);
    path_msg_.header.frame_id = "odom";

    // 控制循环 20Hz (50ms)
    timer_ = this->create_wall_timer(50ms, std::bind(&AdvancedExecutor::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "高级执行节点已启动 (支持语义导航视觉检查)");
  }
  
  ~AdvancedExecutor()
  {
    curl_global_cleanup();
  }

private:
  // 图像回调函数
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    latest_image_ = msg;
    image_received_ = true;
  }
  
  // Odometry 回调函数
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
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

    current_linear_vel_ = msg->twist.twist.linear.x;
    current_angular_vel_ = msg->twist.twist.angular.z;

    odom_received_ = true;

    // 记录轨迹点并发布到 RViz
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = this->now();
    pose.header.frame_id = "odom";
    pose.pose = msg->pose.pose;

    // 每隔一定距离记录一个点
    if (path_msg_.poses.empty() || 
        std::hypot(pose.pose.position.x - path_msg_.poses.back().pose.position.x,
                   pose.pose.position.y - path_msg_.poses.back().pose.position.y) > 0.05) {
      path_msg_.poses.push_back(pose);

      if (path_msg_.poses.size() > 5000) {
        path_msg_.poses.erase(path_msg_.poses.begin());
      }
    }

    path_msg_.header.stamp = this->now();
    pub_path_->publish(path_msg_);
  }

  // 激光雷达回调函数
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    laser_ranges_ = msg->ranges;
    laser_angle_min_ = msg->angle_min;
    laser_angle_increment_ = msg->angle_increment;
    laser_num_points_ = msg->ranges.size();
    scan_received_ = true;
  }

  // ==================== 简单避障算法（来自 avoid 版本）====================
  
  // 避障结果结构体
  struct AvoidResult {
    bool need_avoid;    // 是否需要避障
    double front_dist;  // 前方最近障碍距离
    double turn_dir;    // 转向方向：+1 左转，-1 右转
    double left_avg;    // 左侧平均距离（用于脱困方向判断）
    double right_avg;   // 右侧平均距离
    double rear_min;    // 后方最近障碍距离
  };
  
  // 简单避障：前方有障碍就转向空旷的一侧
  AvoidResult simpleAvoid(double threshold = 0.8)
  {
    AvoidResult result = {false, 10.0, 1.0, 5.0, 5.0, 10.0};
    
    if (!scan_received_ || laser_ranges_.empty()) {
      return result;
    }
    
    int num_points = laser_ranges_.size();
    double front_min = 10.0;  // 前方最近距离
    double rear_min = 10.0;   // 后方最近距离
    double left_sum = 0, right_sum = 0;
    int left_cnt = 0, right_cnt = 0;
    
    for (int i = 0; i < num_points; i++) {
      double angle = laser_angle_min_ + i * laser_angle_increment_;
      double range = laser_ranges_[i];
      
      if (range < 0.05 || range > 10.0) continue;  // 过滤无效数据
      
      // 前方 ±30°
      if (angle > -0.52 && angle < 0.52) {
        if (range < front_min) front_min = range;
      }
      // 左侧 30°~90°
      else if (angle >= 0.52 && angle < 1.57) {
        left_sum += range;
        left_cnt++;
      }
      // 右侧 -90°~-30°
      else if (angle > -1.57 && angle <= -0.52) {
        right_sum += range;
        right_cnt++;
      }
      // 后方 ±30° from 180° (2.79 ~ 3.14 or -3.14 ~ -2.79)
      else if (angle > 2.79 || angle < -2.79) {
        if (range < rear_min) rear_min = range;
      }
    }
    
    result.front_dist = front_min;
    result.rear_min = rear_min;
    result.left_avg = (left_cnt > 0) ? left_sum / left_cnt : 5.0;
    result.right_avg = (right_cnt > 0) ? right_sum / right_cnt : 5.0;
    
    // 前方距离小于阈值，需要避障
    if (front_min < threshold) {
      result.need_avoid = true;
      
      // 哪边更空旷就往哪边转
      result.turn_dir = (result.left_avg >= result.right_avg) ? 1.0 : -1.0;
    }
    
    return result;
  }
  
  // ==================== 视觉检查功能 ====================
  
  // Base64 编码函数
  std::string base64Encode(const unsigned char* data, size_t len)
  {
    std::string ret;
    int i = 0;
    unsigned char char_array_3[3];
    unsigned char char_array_4[4];
    
    while (len--) {
      char_array_3[i++] = *(data++);
      if (i == 3) {
        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
        char_array_4[3] = char_array_3[2] & 0x3f;
        
        for (i = 0; i < 4; i++)
          ret += base64_chars[char_array_4[i]];
        i = 0;
      }
    }
    
    if (i) {
      for (int j = i; j < 3; j++)
        char_array_3[j] = '\0';
      
      char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
      char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
      char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
      
      for (int j = 0; j < i + 1; j++)
        ret += base64_chars[char_array_4[j]];
      
      while (i++ < 3)
        ret += '=';
    }
    
    return ret;
  }
  
  // curl 回调函数
  static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
  {
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
  }
  
  // 视觉检查：判断当前画面是否包含目标
  VisionResult checkVisionTarget(const std::string& target)
  {
    VisionResult result;
    
    if (vision_api_key_.empty()) {
      RCLCPP_WARN(this->get_logger(), "未配置视觉API，跳过视觉检查");
      return result;
    }
    
    if (!image_received_ || !latest_image_) {
      RCLCPP_WARN(this->get_logger(), "未收到图像数据");
      return result;
    }
    
    // 将 ROS Image 转换为 OpenCV Mat
    cv::Mat image;
    try {
      if (latest_image_->encoding == "rgb8") {
        image = cv::Mat(latest_image_->height, latest_image_->width, CV_8UC3, 
                       const_cast<unsigned char*>(latest_image_->data.data()));
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
      } else if (latest_image_->encoding == "bgr8") {
        image = cv::Mat(latest_image_->height, latest_image_->width, CV_8UC3, 
                       const_cast<unsigned char*>(latest_image_->data.data()));
      } else if (latest_image_->encoding == "mono8") {
        image = cv::Mat(latest_image_->height, latest_image_->width, CV_8UC1, 
                       const_cast<unsigned char*>(latest_image_->data.data()));
      } else {
        RCLCPP_WARN(this->get_logger(), "不支持的图像编码: %s", latest_image_->encoding.c_str());
        return result;
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "图像转换失败: %s", e.what());
      return result;
    }
    
    // 编码为 JPEG
    std::vector<uchar> jpeg_buffer;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80};
    if (!cv::imencode(".jpg", image, jpeg_buffer, params)) {
      RCLCPP_ERROR(this->get_logger(), "JPEG 编码失败");
      return result;
    }
    
    // Base64 编码
    std::string base64_image = base64Encode(jpeg_buffer.data(), jpeg_buffer.size());
    
    // 构建 API 请求
    CURL *curl = curl_easy_init();
    if (!curl) {
      RCLCPP_ERROR(this->get_logger(), "CURL 初始化失败");
      return result;
    }
    
    std::string response_string;
    
    // 构建 JSON 请求体
    Json::Value root;
    root["model"] = vision_model_;
    
    Json::Value messages(Json::arrayValue);
    
    // 系统提示
    Json::Value system_msg;
    system_msg["role"] = "system";
    system_msg["content"] = vision_system_prompt_;
    messages.append(system_msg);
    
    // 用户消息（包含图片）
    Json::Value user_msg;
    user_msg["role"] = "user";
    
    Json::Value content(Json::arrayValue);
    
    // 文本部分
    Json::Value text_part;
    text_part["type"] = "text";
    std::string user_prompt = vision_user_prompt_;
    size_t pos = user_prompt.find("{target}");
    if (pos != std::string::npos) {
      user_prompt.replace(pos, 8, target);
    }
    text_part["text"] = user_prompt;
    content.append(text_part);
    
    // 图片部分
    Json::Value image_part;
    image_part["type"] = "image_url";
    Json::Value image_url;
    image_url["url"] = "data:image/jpeg;base64," + base64_image;
    image_part["image_url"] = image_url;
    content.append(image_part);
    
    user_msg["content"] = content;
    messages.append(user_msg);
    
    root["messages"] = messages;
    root["max_tokens"] = 50;
    
    Json::StreamWriterBuilder writer;
    writer["indentation"] = "";
    std::string json_data = Json::writeString(writer, root);
    
    // 设置 HTTP headers
    struct curl_slist *headers = NULL;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    std::string auth_header = "Authorization: Bearer " + vision_api_key_;
    headers = curl_slist_append(headers, auth_header.c_str());
    
    curl_easy_setopt(curl, CURLOPT_URL, vision_api_url_.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_data.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_string);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 15L);
    
    CURLcode res = curl_easy_perform(curl);
    
    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
    
    if (res != CURLE_OK) {
      RCLCPP_WARN(this->get_logger(), "视觉API调用失败: %s", curl_easy_strerror(res));
      return result;
    }
    
    // 解析响应
    Json::Value response_json;
    Json::CharReaderBuilder reader;
    std::string errs;
    std::istringstream response_stream(response_string);
    
    if (Json::parseFromStream(reader, response_stream, &response_json, &errs)) {
      if (response_json.isMember("error")) {
        RCLCPP_ERROR(this->get_logger(), "API错误: %s", 
          response_json["error"]["message"].asString().c_str());
        return result;
      }
      
      if (response_json.isMember("choices") && response_json["choices"].size() > 0) {
        std::string content_str = response_json["choices"][0]["message"]["content"].asString();
        RCLCPP_INFO(this->get_logger(), "🔎 视觉检查返回: %s", content_str.c_str());
        
        // 尝试从响应中提取数字作为置信度
        try {
          std::string num_str;
          for (char c : content_str) {
            if (isdigit(c) || c == '.') num_str += c;
            else if (!num_str.empty()) break;
          }
          if (!num_str.empty()) {
            result.confidence = std::clamp(std::stod(num_str), 0.0, 1.0);
            result.valid = true;
          }
        } catch (const std::exception& e) {
          RCLCPP_WARN(this->get_logger(), "解析置信度失败: %s", e.what());
        }
      }
    }
    
    return result;
  }
  
  // 卡死检测：检查是否长时间位置不变
  bool checkStuck()
  {
    if (!has_active_task_) return false;
    
    double now_sec = this->now().seconds();
    
    // 每0.5秒记录一次位置
    if (now_sec - last_position_check_time_ > 0.5) {
      last_position_check_time_ = now_sec;
      
      double moved = std::hypot(current_x_ - last_check_x_, current_y_ - last_check_y_);
      
      if (moved < 0.02) {  // 0.5秒内移动不到2cm
        stuck_counter_++;
      } else {
        stuck_counter_ = 0;  // 重置计数器
      }
      
      last_check_x_ = current_x_;
      last_check_y_ = current_y_;
      
      // 连续6次（3秒）位置几乎不变，认为卡死
      if (stuck_counter_ >= 6) {
        return true;
      }
    }
    
    return false;
  }
  
  // 执行后退脱困 (优化版：避免重复卡死)
  void executeReverse()
  {
    double reverse_elapsed = (this->now() - reverse_start_time_).seconds();
    
    // 首次进入脱困，记录位置并决定转向策略
    if (!escape_direction_decided_) {
      escape_direction_decided_ = true;
      
      // 检查是否在同一位置多次卡死
      double dist_to_last_stuck = std::hypot(current_x_ - last_stuck_x_, current_y_ - last_stuck_y_);
      if (dist_to_last_stuck < 0.5) {
        // 同一位置多次卡死，增加计数
        same_place_stuck_count_++;
        RCLCPP_WARN(this->get_logger(), "⚠️ 同一位置第 %d 次卡死！", same_place_stuck_count_);
      } else {
        same_place_stuck_count_ = 1;
      }
      last_stuck_x_ = current_x_;
      last_stuck_y_ = current_y_;
      
      // 决定转向方向：如果多次卡死，强制换方向或增大转向
      auto avoid = simpleAvoid();
      if (same_place_stuck_count_ >= 2) {
        // 第2次及以上：与上次相反方向
        escape_turn_left_ = !last_escape_turn_left_;
        escape_turn_intensity_ = 1.0 + 0.3 * same_place_stuck_count_;  // 逐次增大转向
        escape_turn_intensity_ = std::min(escape_turn_intensity_, 2.0);
        RCLCPP_WARN(this->get_logger(), "🔄 切换脱困方向，强度: %.1f", escape_turn_intensity_);
      } else {
        // 第1次：根据空间选择
        escape_turn_left_ = (avoid.left_avg > avoid.right_avg);
        escape_turn_intensity_ = 0.8;
      }
      last_escape_turn_left_ = escape_turn_left_;
    }
    
    if (reverse_elapsed < 1.2) {
      // 阶段1：后退（稍长一点）
      auto avoid = simpleAvoid();
      if (avoid.rear_min > 0.25) {
        // 后退时也带一点转向，避免原路返回
        double w = escape_turn_left_ ? 0.2 : -0.2;
        publishVelocity(-0.25, w);
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
          "🔙 后退脱困中... (%.1fs)", reverse_elapsed);
      } else {
        // 后方也有障碍，跳到转向阶段
        reverse_start_time_ = this->now() - rclcpp::Duration::from_seconds(1.2);
      }
    } else if (reverse_elapsed < 3.0) {
      // 阶段2：边后退边大幅转向
      double w = escape_turn_left_ ? escape_turn_intensity_ : -escape_turn_intensity_;
      publishVelocity(-0.1, w);
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
        "🔄 后退转向中... 往%s转 (强度%.1f)", escape_turn_left_ ? "左" : "右", escape_turn_intensity_);
    } else {
      // 脱困完成
      is_reversing_ = false;
      escape_direction_decided_ = false;
      stuck_counter_ = 0;
      RCLCPP_INFO(this->get_logger(), "✓ 脱困完成，继续导航");
    }
  }

  void cmdCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string incoming_command = msg->data;
    
    if (incoming_command == last_command_) {
      RCLCPP_WARN(this->get_logger(), "检测到重复指令，忽略");
      return;
    }
    
    if (has_active_task_ || !task_queue_.empty()) {
      RCLCPP_WARN(this->get_logger(), 
        "检测到新指令，清空旧任务队列并执行新指令。");
      task_queue_.clear();
      has_active_task_ = false;
      stopRobot();
    }
    
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
          // draw_curve 专用字段
          task.curve_type = item.get("curve_type", "sin").asString();
          task.amplitude = item.get("amplitude", 1.0).asDouble();
          task.wavelength = item.get("wavelength", 2.0).asDouble();
          // 语义导航专用字段
          task.target = item.get("target", "").asString();
          task.confidence = item.get("confidence", 0.80).asDouble();
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
        curve_start_x_ = current_x_;
        curve_start_y_ = current_y_;
        curve_start_yaw_ = current_yaw_;
        // 重置视觉检查状态
        vision_check_pending_ = false;
        vision_check_count_ = 0;
        vision_confidence_sum_ = 0.0;
        vision_check_phase_ = 0;
        vision_confirmed_ = false;
        // 重置转弯状态
        turn_initialized_ = false;
        RCLCPP_INFO(this->get_logger(), "开始执行: %s", current_task_.action.c_str());
      } else {
        stopRobot();
        if (!last_command_.empty()) {
          RCLCPP_INFO(this->get_logger(), "所有任务执行完毕，等待新指令...");
          last_command_.clear();
        }
        return; 
      }
    }

    if (current_task_.action == "move_cmd") executeMoveCmd();
    else if (current_task_.action == "move_to") executeMoveTo();
    else if (current_task_.action == "control_arm") executeArm();
    else if (current_task_.action == "wait") executeWait();
    else if (current_task_.action == "draw_curve") executeDrawCurve();
    else if (current_task_.action == "stop") {
      stopRobot();
      task_queue_.clear();
      has_active_task_ = false;
      last_command_.clear();
      RCLCPP_INFO(this->get_logger(), "已停止");
    }
  }

  void executeMoveCmd()
  {
    // 检测前方障碍物（仅当有前进速度时）
    if (current_task_.linear > 0) {
      auto avoid = simpleAvoid(0.5);
      if (avoid.need_avoid && avoid.front_dist < 0.4) {
        stopRobot();
        task_queue_.clear();
        has_active_task_ = false;
        last_command_.clear();
        RCLCPP_ERROR(this->get_logger(), 
          "❌ move_cmd 前方 %.2fm 有障碍，任务终止！", avoid.front_dist);
        return;
      }
    }
    
    // 判断是否是转弯任务（angular != 0，且为 move_cmd 的定时转弯）
    // 注意：LLM 指令里左/右转通常会带非零 linear（例如 0.5），因此不能用 linear 判断
    bool is_turning = (
      current_task_.action == "move_cmd" &&
      std::abs(current_task_.angular) > 0.1 &&
      current_task_.duration > 0.1
    );
    
    if (is_turning && !turn_initialized_) {
      // 初始化转弯：记录起始角度
      turn_start_yaw_ = current_yaw_;
      // 目标转角：直接使用 angular * duration，保留正负号
      // 正数=左转，负数=右转
      turn_target_angle_ = current_task_.angular * current_task_.duration;
      turn_accumulated_ = 0.0;  // 累计已转角度
      last_yaw_for_turn_ = current_yaw_;  // 上一次的yaw用于增量计算
      
      // 初始化最大速度限制（防止末端反弹加速）
      current_turn_max_w_ = std::abs(current_task_.angular);
      current_turn_max_v_ = 0.08;  // 初始最大线速度
      turn_coasting_ = false;      // 重置滑行状态
      
      turn_initialized_ = true;
      RCLCPP_INFO(this->get_logger(), "🔄 开始转弯: 目标转角=%.1f° (%s)",
        turn_target_angle_ * 180.0 / M_PI, 
        turn_target_angle_ > 0 ? "左转" : "右转");
    }
    
    if (is_turning) {
      // 增量式角度计算：避免±π跨越问题
      double delta_yaw = current_yaw_ - last_yaw_for_turn_;
      // 处理±π跨越
      if (delta_yaw > M_PI) delta_yaw -= 2 * M_PI;
      if (delta_yaw < -M_PI) delta_yaw += 2 * M_PI;
      
      turn_accumulated_ += delta_yaw;
      last_yaw_for_turn_ = current_yaw_;
      
      // 剩余角度 = 目标角度 - 已累计角度
      double remaining = turn_target_angle_ - turn_accumulated_;
      double abs_remaining = std::abs(remaining);
      
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200,
        "🔄 转弯: 目标%.1f°, 已转%.1f°, 剩余%.1f°, 当前角速度=%.3f", 
        turn_target_angle_ * 180.0 / M_PI,
        turn_accumulated_ * 180.0 / M_PI, 
        remaining * 180.0 / M_PI,
        current_angular_vel_);
      
      // ========== 滑行停止策略 ==========
      // 当进入滑行阶段后，完全停止发送速度指令，让机器人靠惯性滑行
      if (turn_coasting_) {
        stopRobot();  // 持续发送零速度
        
        // 检查是否已经基本停止（角速度很小）
        // 注意：必须等机器人真正停下来再结束任务，否则下一个任务会"借用"残留的角速度
        if (std::abs(current_angular_vel_) < 0.01 && std::abs(current_linear_vel_) < 0.01) {
          RCLCPP_INFO(this->get_logger(), "✓ 转弯完成(滑行): 目标%.1f°, 实际%.1f°", 
            turn_target_angle_ * 180.0 / M_PI,
            turn_accumulated_ * 180.0 / M_PI);
          has_active_task_ = false;
          turn_initialized_ = false;
          turn_coasting_ = false;
        }
        return;
      }
      
      // ========== 超调检测 ==========
      // 如果已经转过头了（remaining 变号），立即停止并结束
      bool overshot = (turn_target_angle_ > 0 && remaining < 0) || 
                      (turn_target_angle_ < 0 && remaining > 0);
      if (overshot) {
        stopRobot();
        RCLCPP_WARN(this->get_logger(), "⚠️ 转弯超调! 目标%.1f°, 实际%.1f°, 立即停止", 
          turn_target_angle_ * 180.0 / M_PI,
          turn_accumulated_ * 180.0 / M_PI);
        // 进入滑行模式等待完全停止
        turn_coasting_ = true;
        return;
      }
      
      // 惯性补偿：在剩余角度较大时就停止，让惯性带着完成
      // 根据当前角速度动态计算需要预留的惯性角度
      double inertia_angle = std::abs(current_angular_vel_) * 0.6;  // 预估0.6秒的惯性（再稍微晚停）
      inertia_angle = std::max(inertia_angle, 0.070);  // 至少预留4度
      inertia_angle = std::min(inertia_angle, 0.30);   // 最多预留17度
      
      if (abs_remaining < inertia_angle) {
        turn_coasting_ = true;
        stopRobot();
        RCLCPP_INFO(this->get_logger(), "🛑 进入滑行阶段: 剩余%.1f°, 预留惯性%.1f°", 
          remaining * 180.0 / M_PI, inertia_angle * 180.0 / M_PI);
        return;
      }
      
      // 大幅降低速度，减少惯性
      double proposed_w;
      double proposed_v;
      if (abs_remaining > 1.05) {        // > 60度：较慢速度
        proposed_w = 0.15;               // 大幅降低（原来可能是0.5+）
        proposed_v = 0.04;
      } else {                           // 35~60度：极慢
        proposed_w = 0.10;
        proposed_v = 0.03;
      }
      
      // 关键修正：速度只能单调递减（防抖动/防超调）
      current_turn_max_w_ = std::min(current_turn_max_w_, proposed_w);
      current_turn_max_v_ = std::min(current_turn_max_v_, proposed_v);
      
      double angular_speed = current_turn_max_w_;
      double linear_v = current_turn_max_v_;
      
      // 保持转向方向
      double sign = (remaining > 0) ? 1.0 : -1.0;
      
      publishVelocity(linear_v, sign * angular_speed);
    } else {
      // 非转弯任务：原来的时间控制
      double elapsed = (this->now() - task_start_time_).seconds();
      if (elapsed < current_task_.duration) {
        publishVelocity(current_task_.linear, current_task_.angular);
      } else {
        stopRobot();
        RCLCPP_INFO(this->get_logger(), "✓ move_cmd 完成");
        has_active_task_ = false;
      }
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
    // 首先检查是否卡死，需要后退脱困
    if (is_reversing_) {
      executeReverse();
      return;
    }
    
    // 先计算到目标的距离
    double dx = current_task_.x - current_x_;
    double dy = current_task_.y - current_y_;
    double distance = std::sqrt(dx*dx + dy*dy);
    
    // 判断是否是语义导航（有 target 字段）
    bool is_semantic_nav = !current_task_.target.empty();
    
    // 检测卡死 - 无论语义导航还是普通导航，都后退脱困
    if (checkStuck()) {
      RCLCPP_WARN(this->get_logger(), 
        "⚠️ 检测到卡死！距目标 %.2fm，启动后退脱困...", distance);
      is_reversing_ = true;
      reverse_start_time_ = this->now();
      vision_check_pending_ = false;  // 重置视觉检查状态
      return;
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
      "导航: 当前(%.3f, %.3f) | 目标(%.2f, %.2f) | 距离:%.3f%s%s",
      current_x_, current_y_, current_task_.x, current_task_.y, distance,
      is_semantic_nav ? " [语义]" : "",
      vision_confirmed_ ? " [已确认]" : "");

    // 普通导航到达判断：距离 < 0.1m 即成功
    if (!is_semantic_nav && distance < 0.1) {
      stopRobot();
      RCLCPP_INFO(this->get_logger(), "✓ 到达目标 (%.3f, %.3f)，误差: %.3fm", 
                  current_x_, current_y_, distance);
      has_active_task_ = false;
      stuck_counter_ = 0;
      return;
    }
    
    // 语义导航：距离 < 2.0m 时开始视觉检查（仅确认方向正确，不是到达判断）
    if (is_semantic_nav && distance < 2.0 && !vision_confirmed_) {
      // 开始视觉检查
      if (!vision_check_pending_) {
        vision_check_pending_ = true;
        vision_check_count_ = 0;
        vision_confidence_sum_ = 0.0;
        vision_check_phase_ = 0;  // 0: 减速停止阶段, 1: 检查阶段, 2: 检查完成
        RCLCPP_INFO(this->get_logger(), 
          "🔍 语义导航进入检测区域 (%.2fm < 2.0m)，准备停车进行视觉确认: %s", 
          distance, current_task_.target.c_str());
      }
      
      // 阶段0: 减速停止，准备视觉检查
      if (vision_check_phase_ == 0) {
        stopRobot();  // 完全停止
        vision_check_phase_ = 1;
        RCLCPP_INFO(this->get_logger(), "🛑 已停车，开始视觉确认...");
        return;  // 等待下一个控制周期再检查
      }
      
      // 阶段1: 执行视觉检查（车辆保持静止）
      if (vision_check_phase_ == 1) {
        stopRobot();  // 保持静止
        
        VisionResult vision = checkVisionTarget(current_task_.target);
        if (vision.valid) {
          vision_check_count_++;
          vision_confidence_sum_ += vision.confidence;
          
          double avg_confidence = vision_confidence_sum_ / vision_check_count_;
          RCLCPP_INFO(this->get_logger(), 
            "📊 视觉确认 %d/3: 置信度=%.2f, 平均=%.2f, 阈值=%.2f, 距离=%.2fm",
            vision_check_count_, vision.confidence, avg_confidence, current_task_.confidence, distance);
          
          // 连续检查3次
          if (vision_check_count_ >= 3) {
            vision_check_phase_ = 2;  // 进入检查完成阶段
            vision_check_pending_ = false;
            
            if (avg_confidence >= current_task_.confidence) {
              // 确认是目标，标记已确认，继续前进靠近目标
              vision_confirmed_ = true;
              RCLCPP_INFO(this->get_logger(), 
                "✅ 视觉确认成功！确认前方是 '%s' (置信度 %.2f)，继续前进靠近目标...", 
                current_task_.target.c_str(), avg_confidence);
              // 不return，继续执行导航逻辑前进
            } else {
              // 置信度不够，可能不是目标，但继续前进（可能坐标有偏差）
              RCLCPP_WARN(this->get_logger(), 
                "⚠️ 视觉置信度不足 (%.2f < %.2f)，可能不是目标 '%s'，继续前进尝试靠近...", 
                avg_confidence, current_task_.confidence, current_task_.target.c_str());
              // 不return，继续执行导航逻辑
            }
          } else {
            // 还没检查完3帧，保持静止等待
            return;
          }
        } else {
          // 视觉检查无效（可能API调用中），保持静止等待
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "⏳ 等待视觉确认结果... (%d/3)", vision_check_count_);
          return;
        }
      }
    }
    
    // 根据目标类型确定到达距离阈值
    // 床、沙发、冰箱等大型物体需要更大的到达距离
    double arrival_distance = 0.4;  // 默认到达距离（更近一些）
    double arrival_distance_fallback = 0.2;  // 兜底距离（更近一些）
    
    if (is_semantic_nav) {
      std::string target_lower = current_task_.target;
      std::transform(target_lower.begin(), target_lower.end(), target_lower.begin(), ::tolower);
      
      // 大型物体：床、沙发、冰箱等，到达距离放宽
      if (target_lower.find("bed") != std::string::npos || 
          target_lower.find("sofa") != std::string::npos ||
          target_lower.find("couch") != std::string::npos ||
          target_lower.find("refrigerator") != std::string::npos ||
          target_lower.find("fridge") != std::string::npos) {
        arrival_distance = 0.7;  // 大型物体更接近再算到达
        arrival_distance_fallback = 0.4;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "📦 大型物体目标 '%s'，到达距离放宽到 %.1fm", current_task_.target.c_str(), arrival_distance);
      }
      // 中型家具：桌子、柜子等
      else if (target_lower.find("table") != std::string::npos ||
               target_lower.find("cabinet") != std::string::npos ||
               target_lower.find("wardrobe") != std::string::npos) {
        arrival_distance = 0.55;
        arrival_distance_fallback = 0.35;
      }
    }
    
    // 语义导航到达判断：距离 < arrival_distance 且已通过视觉确认
    if (is_semantic_nav && distance < arrival_distance && vision_confirmed_) {
      stopRobot();
      RCLCPP_INFO(this->get_logger(), 
        "✅ 语义导航完成！已贴近目标 '%s'，距离: %.2fm (阈值: %.1fm)", 
        current_task_.target.c_str(), distance, arrival_distance);
      has_active_task_ = false;
      stuck_counter_ = 0;
      vision_check_pending_ = false;
      vision_confirmed_ = false;  // 重置
      return;
    }
    
    // 兜底：语义导航距离 < arrival_distance_fallback 无论如何都算到达
    if (is_semantic_nav && distance < arrival_distance_fallback) {
      stopRobot();
      RCLCPP_INFO(this->get_logger(), 
        "✅ 语义导航完成（兜底）！已非常接近目标 '%s'，距离: %.2fm", 
        current_task_.target.c_str(), distance);
      has_active_task_ = false;
      stuck_counter_ = 0;
      vision_check_pending_ = false;
      vision_confirmed_ = false;
      return;
    }

    double v = 0.0;
    double w = 0.0;
    
    const double wheelbase = 0.22;
    const double max_steer = 0.6109;

    // 计算目标方向
    double target_angle = std::atan2(dy, dx);
    double alpha = target_angle - current_yaw_;
    while (alpha > M_PI) alpha -= 2 * M_PI;
    while (alpha < -M_PI) alpha += 2 * M_PI;

    if (distance < 0.3) {
      // 近距离精细控制
      v = std::clamp(1.5 * distance, 0.05, 0.15);
      if (std::abs(alpha) > 0.3) v *= 0.6;

      double delta = std::atan(wheelbase * std::sin(alpha) / (distance * 0.5 + 0.05));
      delta = std::clamp(delta, -max_steer, max_steer);
      w = v * std::tan(delta) / wheelbase;
    } else {
      // Pure Pursuit 远距离控制
      const double min_lookahead = 0.15;
      const double max_lookahead = 0.5;
      const double lookahead = std::clamp(0.4 * distance, min_lookahead, max_lookahead);

      v = std::clamp(0.8 * distance, 0.1, 0.4);

      double delta = std::atan(2.0 * wheelbase * std::sin(alpha) / lookahead);
      delta = std::clamp(delta, -max_steer, max_steer);
      w = v * std::tan(delta) / wheelbase;
    }

    // Ackermann 约束：转向时必须有前进速度
    if (std::abs(w) > 0.05 && v < 0.05) {
      v = 0.05;
    }

    // ==================== 简单避障检测 ====================
    // 关键优化：视觉确认后，直接往前开，用激光雷达检测前方距离来判断到达
    
    if (is_semantic_nav && vision_confirmed_) {
      // 已确认是目标，进入直接接近模式
      auto avoid = simpleAvoid(0.5);  // 只是获取前方距离，不用于避障
      double front_dist = avoid.front_dist;
      
      // 根据目标类型确定停止距离
      double stop_distance = 0.4;  // 默认停止距离（更近一些）
      std::string target_lower = current_task_.target;
      std::transform(target_lower.begin(), target_lower.end(), target_lower.begin(), ::tolower);
      
      if (target_lower.find("bed") != std::string::npos || 
          target_lower.find("sofa") != std::string::npos ||
          target_lower.find("refrigerator") != std::string::npos ||
          target_lower.find("fridge") != std::string::npos) {
        stop_distance = 0.45;  // 大型物体也更近一些
      } else if (target_lower.find("table") != std::string::npos ||
                 target_lower.find("cabinet") != std::string::npos) {
        stop_distance = 0.35;
      }
      
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "🎯 接近目标 '%s': 前方障碍距离=%.2fm, 停止距离=%.2fm", 
        current_task_.target.c_str(), front_dist, stop_distance);
      
      if (front_dist < stop_distance) {
        // 前方距离足够近，认为已到达目标，停车！
        stopRobot();
        RCLCPP_INFO(this->get_logger(), 
          "✅ 语义导航完成！激光雷达检测前方 %.2fm，已贴近目标 '%s'", 
          front_dist, current_task_.target.c_str());
        has_active_task_ = false;
        stuck_counter_ = 0;
        vision_check_pending_ = false;
        vision_confirmed_ = false;
        return;
      }
      
      // 还没到，继续直线前进（速度根据距离递减）
      double approach_v = std::clamp((front_dist - stop_distance) * 0.5, 0.05, 0.22);
      publishVelocity(approach_v, w);  // 保持原来的转向角速度w，只调整线速度
      return;  // 直接返回，不进入后面的避障逻辑
    }
    
    // 未确认或不是语义导航，正常避障
    double avoid_threshold = 0.8;  // 默认避障阈值
    const double MIN_SAFE_DISTANCE = 0.15;  // 最小安全距离（稍微放宽）
    
    if (is_semantic_nav) {
      // 语义导航但未确认：根据距离调整避障阈值
      if (distance < 1.0) {
        avoid_threshold = 0.25;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "🎯 接近目标 (%.2fm)，降低避障阈值到 %.2fm", distance, avoid_threshold);
      } else if (distance < 2.0) {
        avoid_threshold = 0.35;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "🎯 靠近目标 (%.2fm)，降低避障阈值到 %.2fm", distance, avoid_threshold);
      }
    }
    
    auto avoid = simpleAvoid(avoid_threshold);
    
    // 检查最小安全距离
    if (avoid.front_dist < MIN_SAFE_DISTANCE && v > 0) {
      v = 0.0;
      w = 0.0;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "🛑 前方 %.2fm 有障碍物，紧急停止！", avoid.front_dist);
    } else if (avoid.need_avoid && v > 0) {
      if (avoid.front_dist < 0.35) {
        v = 0.06;
      } else if (avoid.front_dist < 0.5) {
        v = 0.12;
      } else {
        v = 0.20;
      }
      w = avoid.turn_dir * 0.45;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "🚗 避障: 前方=%.2fm, 转向=%s, v=%.2f, w=%.2f",
        avoid.front_dist, avoid.turn_dir > 0 ? "左" : "右", v, w);
    }

    publishVelocity(v, w);
  }

  // 绘制曲线
  void executeDrawCurve()
  {
    double elapsed = (this->now() - task_start_time_).seconds();
    
    if (elapsed >= current_task_.duration) {
      stopRobot();
      RCLCPP_INFO(this->get_logger(), "draw_curve 完成");
      has_active_task_ = false;
      return;
    }
    
    double v = 0.3;  // 基础前进速度
    double w = 0.0;
    
    if (current_task_.curve_type == "sin") {
      // sin 曲线：角速度随时间正弦变化
      double freq = 2 * M_PI / current_task_.wavelength;  // 频率
      w = current_task_.amplitude * std::sin(freq * elapsed * v);
    } else if (current_task_.curve_type == "cos") {
      double freq = 2 * M_PI / current_task_.wavelength;
      w = current_task_.amplitude * std::cos(freq * elapsed * v);
    } else if (current_task_.curve_type == "circle") {
      // 画圆：恒定角速度
      w = v / current_task_.amplitude;  // amplitude 作为半径
    }
    
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
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_arm_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  nav_msgs::msg::Path path_msg_;

  std::deque<Task> task_queue_;
  Task current_task_;
  bool has_active_task_ = false;
  rclcpp::Time task_start_time_;

  double current_x_ = 0.0;
  double current_y_ = 0.0;
  double current_yaw_ = 0.0;
  
  double current_linear_vel_ = 0.0;
  double current_angular_vel_ = 0.0;
  
  bool odom_received_ = false;
  
  std::vector<float> laser_ranges_;
  double laser_angle_min_ = 0.0;
  double laser_angle_increment_ = 0.0;
  size_t laser_num_points_ = 0;
  bool scan_received_ = false;
  
  // 图像相关
  sensor_msgs::msg::Image::SharedPtr latest_image_;
  bool image_received_ = false;
  
  // 视觉API配置
  std::string vision_api_key_;
  std::string vision_api_url_;
  std::string vision_model_;
  std::string vision_system_prompt_;
  std::string vision_user_prompt_;
  
  // 视觉检查状态
  bool vision_check_pending_ = false;
  int vision_check_count_ = 0;
  double vision_confidence_sum_ = 0.0;
  int vision_check_phase_ = 0;  // 0: 减速停止阶段, 1: 检查阶段, 2: 检查完成
  bool vision_confirmed_ = false;  // 视觉确认通过，可以继续前进靠近目标
  
  // 卡死检测相关
  int stuck_counter_ = 0;
  double last_position_check_time_ = 0.0;
  double last_check_x_ = 0.0;
  double last_check_y_ = 0.0;
  
  // 后退脱困相关
  bool is_reversing_ = false;
  rclcpp::Time reverse_start_time_;
  bool escape_direction_decided_ = false;  // 是否已决定脱困方向
  bool escape_turn_left_ = true;           // 脱困时往左转还是右转
  bool last_escape_turn_left_ = true;      // 上次脱困的转向方向
  double escape_turn_intensity_ = 0.8;     // 脱困转向强度
  double last_stuck_x_ = 0.0;              // 上次卡死位置X
  double last_stuck_y_ = 0.0;              // 上次卡死位置Y
  int same_place_stuck_count_ = 0;         // 同一位置卡死次数
  
  std::string last_command_;
  
  // draw_curve 起点记录
  double curve_start_x_ = 0.0;
  double curve_start_y_ = 0.0;
  double curve_start_yaw_ = 0.0;
  
  // 转弯闭环控制相关
  bool turn_initialized_ = false;
  double turn_start_yaw_ = 0.0;
  double turn_target_angle_ = 0.0;
  double turn_accumulated_ = 0.0;   // 累计已转角度（增量式计算）
  double last_yaw_for_turn_ = 0.0;  // 上一次yaw值，用于增量计算
  double current_turn_max_w_ = 0.0; // 转弯最大角速度限制（防止末端加速抖动）
  double current_turn_max_v_ = 0.0; // 转弯最大线速度限制（防止末端加速抖动）
  bool turn_coasting_ = false;      // 是否进入滑行停止阶段
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AdvancedExecutor>());
  rclcpp::shutdown();
  return 0;
}
