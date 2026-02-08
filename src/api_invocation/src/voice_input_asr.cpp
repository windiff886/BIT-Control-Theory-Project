/**
 * @file voice_input_asr.cpp
 * @brief 真正的语音识别输入节点 - 使用麦克风录音 + 百度语音识别API
 * 
 * 依赖: libasound2-dev, libcurl4-openssl-dev
 * 安装: sudo apt install libasound2-dev libcurl4-openssl-dev
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <cstring>
#include <fstream>
#include <sstream>
#include <cmath>
#include <alsa/asoundlib.h>
#include <curl/curl.h>
#include <json/json.h>

// Base64 编码表
static const char base64_chars[] = 
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

class VoiceInputASRNode : public rclcpp::Node
{
public:
  VoiceInputASRNode() : Node("voice_input_asr_node")
  {
    // 声明参数（百度语音识别API配置）
    this->declare_parameter<std::string>("baidu_api_key", "");
    this->declare_parameter<std::string>("baidu_secret_key", "");
    this->declare_parameter<std::string>("audio_device", "default");
    this->declare_parameter<int>("sample_rate", 16000);
    this->declare_parameter<int>("record_seconds", 5);
    this->declare_parameter<double>("silence_threshold", 500.0);
    
    // 获取参数
    api_key_ = this->get_parameter("baidu_api_key").as_string();
    secret_key_ = this->get_parameter("baidu_secret_key").as_string();
    audio_device_ = this->get_parameter("audio_device").as_string();
    sample_rate_ = this->get_parameter("sample_rate").as_int();
    record_seconds_ = this->get_parameter("record_seconds").as_int();
    silence_threshold_ = this->get_parameter("silence_threshold").as_double();
    
    // 创建发布者
    voice_command_pub_ = this->create_publisher<std_msgs::msg::String>("voice_command", 10);
    
    // 初始化 CURL
    curl_global_init(CURL_GLOBAL_ALL);
    
    printBanner();
    
    // 检查API配置
    if (api_key_.empty() || secret_key_.empty()) {
      RCLCPP_WARN(this->get_logger(), 
        "⚠️  未配置百度语音API，将使用模拟模式（按回车后输入文字）");
      RCLCPP_INFO(this->get_logger(), 
        "   配置方法: ros2 run api_invocation voice_input_asr "
        "--ros-args -p baidu_api_key:=xxx -p baidu_secret_key:=xxx");
      use_simulation_ = true;
    } else {
      // 获取Access Token
      if (!getAccessToken()) {
        RCLCPP_ERROR(this->get_logger(), "获取百度API Access Token失败，切换到模拟模式");
        use_simulation_ = true;
      } else {
        RCLCPP_INFO(this->get_logger(), "✓ 百度语音识别API初始化成功");
        use_simulation_ = false;
      }
    }
    
    // 启动输入线程
    input_thread_ = std::thread(&VoiceInputASRNode::inputLoop, this);
  }
  
  ~VoiceInputASRNode()
  {
    running_ = false;
    if (input_thread_.joinable()) {
      input_thread_.join();
    }
    curl_global_cleanup();
  }

private:
  void printBanner()
  {
    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════════════════════╗\n";
    std::cout << "║          🎤 语音识别导航命令输入终端 🎤              ║\n";
    std::cout << "╠══════════════════════════════════════════════════════╣\n";
    std::cout << "║  使用方法:                                           ║\n";
    std::cout << "║   按 [Enter] 开始录音 (录音 " << record_seconds_ << " 秒)                    ║\n";
    std::cout << "║   按 [t] + [Enter] 切换到文字输入模式                ║\n";
    std::cout << "║   按 [q] + [Enter] 退出                              ║\n";
    std::cout << "╠══════════════════════════════════════════════════════╣\n";
    std::cout << "║  支持的指令类型:                                     ║\n";
    std::cout << "║   • \"去坐标(2,2)\" | \"走到两个球中间\"                 ║\n";
    std::cout << "║   • \"向前走3秒\" | \"左转90度\"                        ║\n";
    std::cout << "║   • \"画一条sinx曲线\" | \"停止\"                       ║\n";
    std::cout << "╚══════════════════════════════════════════════════════╝\n";
    std::cout << std::endl;
  }

  void inputLoop()
  {
    std::string input;
    bool text_mode = use_simulation_;
    
    while (running_ && rclcpp::ok()) {
      if (text_mode || use_simulation_) {
        // 文字输入模式
        std::cout << "\n📝 请输入指令 (或按Enter开始语音录音)> " << std::flush;
      } else {
        // 语音输入模式
        std::cout << "\n🎤 按 [Enter] 开始语音录音 (或输入 't' 切换文字模式)> " << std::flush;
      }
      
      if (!std::getline(std::cin, input)) {
        break;
      }
      
      // 去除首尾空白
      input.erase(0, input.find_first_not_of(" \t\n\r"));
      if (!input.empty()) {
        input.erase(input.find_last_not_of(" \t\n\r") + 1);
      }
      
      // 检查退出命令
      if (input == "quit" || input == "exit" || input == "q") {
        RCLCPP_INFO(this->get_logger(), "正在退出...");
        running_ = false;
        rclcpp::shutdown();
        break;
      }
      
      // 切换模式
      if (input == "t" || input == "text") {
        text_mode = true;
        std::cout << "✓ 已切换到文字输入模式\n";
        continue;
      }
      if (input == "v" || input == "voice") {
        if (use_simulation_) {
          std::cout << "⚠️  未配置语音API，无法使用语音模式\n";
        } else {
          text_mode = false;
          std::cout << "✓ 已切换到语音输入模式\n";
        }
        continue;
      }
      
      std::string recognized_text;
      
      if (input.empty() && !use_simulation_ && !text_mode) {
        // 空输入 = 开始录音
        recognized_text = recordAndRecognize();
      } else if (!input.empty()) {
        // 有文字输入就直接使用
        recognized_text = input;
      } else {
        // 模拟模式下空输入，提示用户
        std::cout << "请输入指令文字\n";
        continue;
      }
      
      if (recognized_text.empty()) {
        std::cout << "❌ 未识别到有效语音，请重试\n";
        continue;
      }
      
      // 发布指令
      auto msg = std_msgs::msg::String();
      msg.data = recognized_text;
      voice_command_pub_->publish(msg);
      
      RCLCPP_INFO(this->get_logger(), "✓ 已发送指令: \"%s\"", recognized_text.c_str());
      RCLCPP_INFO(this->get_logger(), "  等待LLM解析中...");
    }
  }

  /**
   * @brief 录音并进行语音识别
   */
  std::string recordAndRecognize()
  {
    std::cout << "\n🔴 正在录音... (请说话，" << record_seconds_ << "秒后自动停止)\n" << std::flush;
    
    // 录音
    std::vector<int16_t> audio_data;
    if (!recordAudio(audio_data)) {
      RCLCPP_ERROR(this->get_logger(), "录音失败");
      return "";
    }
    
    std::cout << "⏹️  录音结束，正在识别...\n" << std::flush;
    
    // 语音识别
    std::string result = recognizeSpeech(audio_data);
    
    if (!result.empty()) {
      std::cout << "🗣️  识别结果: \"" << result << "\"\n";
    }
    
    return result;
  }

  /**
   * @brief 使用ALSA录音
   */
  bool recordAudio(std::vector<int16_t>& audio_data)
  {
    snd_pcm_t* capture_handle;
    snd_pcm_hw_params_t* hw_params;
    
    int err;
    
    // 打开音频设备
    if ((err = snd_pcm_open(&capture_handle, audio_device_.c_str(), 
                            SND_PCM_STREAM_CAPTURE, 0)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "无法打开音频设备 '%s': %s", 
                   audio_device_.c_str(), snd_strerror(err));
      RCLCPP_INFO(this->get_logger(), "提示: 使用 'arecord -l' 查看可用设备");
      return false;
    }
    
    // 分配硬件参数结构
    snd_pcm_hw_params_alloca(&hw_params);
    snd_pcm_hw_params_any(capture_handle, hw_params);
    
    // 设置参数
    snd_pcm_hw_params_set_access(capture_handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(capture_handle, hw_params, SND_PCM_FORMAT_S16_LE);
    snd_pcm_hw_params_set_channels(capture_handle, hw_params, 1);  // 单声道
    
    unsigned int rate = sample_rate_;
    snd_pcm_hw_params_set_rate_near(capture_handle, hw_params, &rate, 0);
    
    if ((err = snd_pcm_hw_params(capture_handle, hw_params)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "无法设置硬件参数: %s", snd_strerror(err));
      snd_pcm_close(capture_handle);
      return false;
    }
    
    snd_pcm_prepare(capture_handle);
    
    // 计算需要的帧数
    int frames_per_period = 1024;
    int total_frames = sample_rate_ * record_seconds_;
    int periods = total_frames / frames_per_period;
    
    audio_data.reserve(total_frames);
    std::vector<int16_t> buffer(frames_per_period);
    
    // 录音
    for (int i = 0; i < periods && running_; ++i) {
      int frames_read = snd_pcm_readi(capture_handle, buffer.data(), frames_per_period);
      if (frames_read < 0) {
        frames_read = snd_pcm_recover(capture_handle, frames_read, 0);
      }
      if (frames_read > 0) {
        audio_data.insert(audio_data.end(), buffer.begin(), buffer.begin() + frames_read);
      }
      
      // 显示进度
      int progress = (i + 1) * 100 / periods;
      std::cout << "\r🔴 录音中... [";
      int bar_width = 30;
      int pos = bar_width * progress / 100;
      for (int j = 0; j < bar_width; ++j) {
        if (j < pos) std::cout << "█";
        else std::cout << "░";
      }
      std::cout << "] " << progress << "%" << std::flush;
    }
    std::cout << "\n";
    
    snd_pcm_close(capture_handle);
    return true;
  }

  /**
   * @brief 获取百度API Access Token
   */
  bool getAccessToken()
  {
    CURL* curl = curl_easy_init();
    if (!curl) return false;
    
    std::string url = "https://aip.baidubce.com/oauth/2.0/token?"
                      "grant_type=client_credentials&"
                      "client_id=" + api_key_ + "&"
                      "client_secret=" + secret_key_;
    
    std::string response;
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curlWriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 10L);
    
    CURLcode res = curl_easy_perform(curl);
    curl_easy_cleanup(curl);
    
    if (res != CURLE_OK) {
      RCLCPP_ERROR(this->get_logger(), "获取Token失败: %s", curl_easy_strerror(res));
      return false;
    }
    
    // 解析JSON响应
    Json::Value root;
    Json::CharReaderBuilder builder;
    std::istringstream stream(response);
    std::string errors;
    
    if (!Json::parseFromStream(builder, stream, &root, &errors)) {
      RCLCPP_ERROR(this->get_logger(), "解析Token响应失败");
      return false;
    }
    
    if (root.isMember("access_token")) {
      access_token_ = root["access_token"].asString();
      return true;
    }
    
    return false;
  }

  /**
   * @brief 调用百度语音识别API
   */
  std::string recognizeSpeech(const std::vector<int16_t>& audio_data)
  {
    if (access_token_.empty()) {
      return "";
    }
    
    // 将音频数据转换为Base64
    std::string pcm_data(reinterpret_cast<const char*>(audio_data.data()), 
                         audio_data.size() * sizeof(int16_t));
    std::string base64_audio = base64Encode(pcm_data);
    
    // 构建请求JSON
    Json::Value request;
    request["format"] = "pcm";
    request["rate"] = sample_rate_;
    request["channel"] = 1;
    request["cuid"] = "ros2_voice_input";
    request["token"] = access_token_;
    request["speech"] = base64_audio;
    request["len"] = static_cast<int>(pcm_data.size());
    
    Json::StreamWriterBuilder writer;
    std::string request_body = Json::writeString(writer, request);
    
    // 发送请求
    CURL* curl = curl_easy_init();
    if (!curl) return "";
    
    std::string url = "https://vop.baidu.com/server_api";
    std::string response;
    
    struct curl_slist* headers = NULL;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, request_body.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curlWriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30L);
    
    CURLcode res = curl_easy_perform(curl);
    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
    
    if (res != CURLE_OK) {
      RCLCPP_ERROR(this->get_logger(), "语音识别请求失败: %s", curl_easy_strerror(res));
      return "";
    }
    
    // 解析响应
    Json::Value root;
    Json::CharReaderBuilder builder;
    std::istringstream stream(response);
    std::string errors;
    
    if (!Json::parseFromStream(builder, stream, &root, &errors)) {
      RCLCPP_ERROR(this->get_logger(), "解析识别结果失败");
      return "";
    }
    
    if (root.isMember("err_no") && root["err_no"].asInt() == 0) {
      if (root.isMember("result") && root["result"].isArray() && root["result"].size() > 0) {
        return root["result"][0].asString();
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "语音识别错误: %s", 
                   root.isMember("err_msg") ? root["err_msg"].asString().c_str() : "未知错误");
    }
    
    return "";
  }

  /**
   * @brief Base64 编码
   */
  std::string base64Encode(const std::string& input)
  {
    std::string output;
    int i = 0;
    unsigned char char_array_3[3];
    unsigned char char_array_4[4];
    size_t in_len = input.size();
    const char* bytes_to_encode = input.c_str();
    
    while (in_len--) {
      char_array_3[i++] = *(bytes_to_encode++);
      if (i == 3) {
        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
        char_array_4[3] = char_array_3[2] & 0x3f;
        
        for (i = 0; i < 4; i++) {
          output += base64_chars[char_array_4[i]];
        }
        i = 0;
      }
    }
    
    if (i) {
      for (int j = i; j < 3; j++) {
        char_array_3[j] = '\0';
      }
      
      char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
      char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
      char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
      
      for (int j = 0; j < i + 1; j++) {
        output += base64_chars[char_array_4[j]];
      }
      
      while (i++ < 3) {
        output += '=';
      }
    }
    
    return output;
  }

  static size_t curlWriteCallback(void* contents, size_t size, size_t nmemb, void* userp)
  {
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
  }

  // 成员变量
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr voice_command_pub_;
  std::thread input_thread_;
  std::atomic<bool> running_{true};
  
  // API配置
  std::string api_key_;
  std::string secret_key_;
  std::string access_token_;
  std::string audio_device_;
  int sample_rate_;
  int record_seconds_;
  double silence_threshold_;
  bool use_simulation_ = false;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VoiceInputASRNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
