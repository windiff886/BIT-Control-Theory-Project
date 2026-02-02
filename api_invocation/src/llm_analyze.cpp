#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <curl/curl.h>
#include <string>
#include <sstream>
#include <json/json.h>

class LLMAnalyzer : public rclcpp::Node
{
public:
  LLMAnalyzer() : Node("llm_analyzer")
  {
    // 声明参数
    this->declare_parameter("api_key", "YOUR_API_KEY_HERE");
    this->declare_parameter("api_url", "https://api.openai.com/v1/chat/completions");
    this->declare_parameter("model", "gpt-3.5-turbo");
    this->declare_parameter("system_prompt", "");
    this->declare_parameter("temperature", 0.7);
    this->declare_parameter("max_tokens", 2000);
    
    // 获取参数
    api_key_ = this->get_parameter("api_key").as_string();
    api_url_ = this->get_parameter("api_url").as_string();
    model_ = this->get_parameter("model").as_string();
    system_prompt_ = this->get_parameter("system_prompt").as_string();
    temperature_ = this->get_parameter("temperature").as_double();
    max_tokens_ = this->get_parameter("max_tokens").as_int();
    
    // 创建订阅者 - 订阅自然语言指令
    voice_command_sub_ = this->create_subscription<std_msgs::msg::String>(
      "voice_command", 10,
      std::bind(&LLMAnalyzer::voiceCommandCallback, this, std::placeholders::_1));
    
    // 创建发布者 - 发布LLM解析后的导航指令
    navigation_command_pub_ = this->create_publisher<std_msgs::msg::String>(
      "navigation_command", 10);
    
    // 初始化curl
    curl_global_init(CURL_GLOBAL_DEFAULT);
    
    RCLCPP_INFO(this->get_logger(), "LLM Analyzer node initialized");
    RCLCPP_INFO(this->get_logger(), "Listening for voice commands on topic: voice_command");
    RCLCPP_INFO(this->get_logger(), "Publishing navigation commands on topic: navigation_command");
  }
  
  ~LLMAnalyzer()
  {
    curl_global_cleanup();
  }

private:
  // curl回调函数,用于接收响应数据
  static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
  {
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
  }
  
  // 调用LLM API
  std::string callLLMAPI(const std::string &user_input)
  {
    CURL *curl;
    CURLcode res;
    std::string response_string;
    
    curl = curl_easy_init();
    if(curl) {
      // 构建JSON请求体
      Json::Value root;
      root["model"] = model_;
      
      Json::Value messages(Json::arrayValue);
      
      // 添加系统提示词(如果有)
      if (!system_prompt_.empty()) {
        Json::Value system_msg;
        system_msg["role"] = "system";
        system_msg["content"] = system_prompt_;
        messages.append(system_msg);
      }
      
      // 添加用户输入
      Json::Value user_msg;
      user_msg["role"] = "user";
      user_msg["content"] = user_input;
      messages.append(user_msg);
      
      root["messages"] = messages;
      root["temperature"] = temperature_;
      root["max_tokens"] = max_tokens_;
      
      // 使用紧凑的JSON格式
      Json::StreamWriterBuilder writer;
      writer["indentation"] = "";
      std::string json_data = Json::writeString(writer, root);
      
      RCLCPP_DEBUG(this->get_logger(), "Request JSON: %s", json_data.c_str());
      
      // 设置HTTP headers
      struct curl_slist *headers = NULL;
      headers = curl_slist_append(headers, "Content-Type: application/json");
      std::string auth_header = "Authorization: Bearer " + api_key_;
      headers = curl_slist_append(headers, auth_header.c_str());
      
      // 设置curl选项
      curl_easy_setopt(curl, CURLOPT_URL, api_url_.c_str());
      curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
      curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_data.c_str());
      curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
      curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_string);
      curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30L);
      
      // 执行请求
      RCLCPP_INFO(this->get_logger(), "Sending request to LLM API...");
      res = curl_easy_perform(curl);
      
      if(res != CURLE_OK) {
        RCLCPP_ERROR(this->get_logger(), "curl_easy_perform() failed: %s", 
                     curl_easy_strerror(res));
        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);
        return "";
      }
      
      // 清理
      curl_slist_free_all(headers);
      curl_easy_cleanup(curl);
      
      // 解析JSON响应
      Json::Value response_json;
      Json::CharReaderBuilder reader;
      std::string errs;
      std::istringstream response_stream(response_string);
      
      RCLCPP_DEBUG(this->get_logger(), "Response: %s", response_string.c_str());
      
      if (Json::parseFromStream(reader, response_stream, &response_json, &errs)) {
        // 检查是否有错误
        if (response_json.isMember("error")) {
          RCLCPP_ERROR(this->get_logger(), "API Error: %s", 
                       response_json["error"]["message"].asString().c_str());
          return "";
        }
        
        // 提取响应内容
        if (response_json.isMember("choices") && 
            response_json["choices"].isArray() && 
            response_json["choices"].size() > 0) {
          
          const Json::Value& choice = response_json["choices"][0];
          const Json::Value& message = choice["message"];
          
          if (message.isMember("content")) {
            std::string content = message["content"].asString();
            RCLCPP_INFO(this->get_logger(), "✓ LLM response received");
            
            // 打印token使用情况(如果有)
            if (response_json.isMember("usage")) {
              int total_tokens = response_json["usage"]["total_tokens"].asInt();
              RCLCPP_DEBUG(this->get_logger(), "Tokens used: %d", total_tokens);
            }
            
            return content;
          }
        }
        
        RCLCPP_ERROR(this->get_logger(), "Unexpected response format");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON response: %s", errs.c_str());
        RCLCPP_ERROR(this->get_logger(), "Raw response: %s", response_string.c_str());
      }
    }
    
    return "";
  }
  
  // 语音指令回调函数
  void voiceCommandCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received voice command: '%s'", msg->data.c_str());
    
    // 调用LLM API进行分析
    std::string llm_response = callLLMAPI(msg->data);
    
    if (!llm_response.empty()) {
      // 发布LLM的分析结果
      auto navigation_msg = std_msgs::msg::String();
      navigation_msg.data = llm_response;
      navigation_command_pub_->publish(navigation_msg);
      
      RCLCPP_INFO(this->get_logger(), "Published navigation command: '%s'", 
                  llm_response.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to get valid response from LLM");
    }
  }
  
  // 成员变量
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr voice_command_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr navigation_command_pub_;
  
  std::string api_key_;
  std::string api_url_;
  std::string model_;
  std::string system_prompt_;
  double temperature_;
  int max_tokens_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LLMAnalyzer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
