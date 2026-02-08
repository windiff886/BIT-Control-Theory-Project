#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <curl/curl.h>
#include <string>
#include <sstream>
#include <iomanip>
#include <map>
#include <set>
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
    base_system_prompt_ = this->get_parameter("system_prompt").as_string();
    temperature_ = this->get_parameter("temperature").as_double();
    max_tokens_ = this->get_parameter("max_tokens").as_int();
    
    // 创建订阅者 - 订阅自然语言指令
    voice_command_sub_ = this->create_subscription<std_msgs::msg::String>(
      "voice_command", 10,
      std::bind(&LLMAnalyzer::voiceCommandCallback, this, std::placeholders::_1));
    
    // 订阅模型状态话题 (由 model_state_publisher 发布)
    model_states_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/gazebo/model_states", 10,
      std::bind(&LLMAnalyzer::modelStatesCallback, this, std::placeholders::_1));
    
    // 创建发布者 - 发布LLM解析后的导航指令
    navigation_command_pub_ = this->create_publisher<std_msgs::msg::String>(
      "navigation_command", 10);
    
    // 初始化curl
    curl_global_init(CURL_GLOBAL_DEFAULT);
    
    // 初始化模型名称映射
    initModelNameMapping();
    
    RCLCPP_INFO(this->get_logger(), "LLM Analyzer node initialized");
    RCLCPP_INFO(this->get_logger(), "Listening for voice commands on topic: voice_command");
    RCLCPP_INFO(this->get_logger(), "Listening for model states on topic: /gazebo/model_states");
    RCLCPP_INFO(this->get_logger(), "Publishing navigation commands on topic: navigation_command");
  }
  
  ~LLMAnalyzer()
  {
    curl_global_cleanup();
  }

private:
  // 初始化模型名称映射
  void initModelNameMapping()
  {
    // Gazebo模型名 -> 中文名称
    model_name_map_["Ball_01_001"] = "红色球1 (Ball 1)";
    model_name_map_["Ball_01_002"] = "红色球2 (Ball 2)";
    model_name_map_["Ball_01_003"] = "红色球3 (Ball 3)";
    model_name_map_["Bed_01_001"] = "床 (Bed)";
    model_name_map_["Bed_01"] = "床 (Bed)";
    model_name_map_["NightStand_01_001"] = "床头柜1 (NightStand 1)";
    model_name_map_["NightStand_01_002"] = "床头柜2 (NightStand 2)";
    model_name_map_["CoffeeTable_01_001"] = "茶几 (Coffee Table)";
    model_name_map_["CoffeeTable_01"] = "茶几 (Coffee Table)";
    model_name_map_["KitchenTable_01_001"] = "餐桌 (Kitchen Table)";
    model_name_map_["KitchenTable_01"] = "餐桌 (Kitchen Table)";
    model_name_map_["Refrigerator_01_001"] = "冰箱 (Refrigerator)";
    model_name_map_["Refrigerator_01"] = "冰箱 (Refrigerator)";
    model_name_map_["KitchenCabinet_01_001"] = "橱柜 (Kitchen Cabinet)";
    model_name_map_["KitchenCabinet_01"] = "橱柜 (Kitchen Cabinet)";
    model_name_map_["ChairA_01_001"] = "椅子 (Chair)";
    model_name_map_["ChairA_01"] = "椅子 (Chair)";
    model_name_map_["ChairD_01_001"] = "餐椅 (Dining Chair)";
    model_name_map_["ChairD_01"] = "餐椅 (Dining Chair)";
    model_name_map_["FoldingDoor_01"] = "折叠门 (Folding Door)";
    model_name_map_["Door_01"] = "门 (Door)";
    model_name_map_["Curtain_01"] = "窗帘 (Curtain)";
    
    // 需要过滤的模型 (地面、墙壁、机器人本身等)
    filter_models_.insert("ground_plane");
    filter_models_.insert("simple_ground_house");
    filter_models_.insert("HouseWallB_01");
    filter_models_.insert("FloorB_01");
    filter_models_.insert("ackermann_steering_vehicle");
    filter_models_.insert("sun");
  }
  
  // 判断是否应该过滤该模型
  bool shouldFilterModel(const std::string& model_name)
  {
    if (filter_models_.find(model_name) != filter_models_.end()) {
      return true;
    }
    // 过滤掉墙壁、地板、窗户等
    if (model_name.find("Wall") != std::string::npos ||
        model_name.find("Floor") != std::string::npos ||
        model_name.find("Window") != std::string::npos ||
        model_name.find("Light") != std::string::npos ||
        model_name.find("Chandelier") != std::string::npos ||
        model_name.find("Portrait") != std::string::npos ||
        model_name.find("Handle") != std::string::npos ||
        model_name.find("Carpet") != std::string::npos ||
        model_name.find("ground") != std::string::npos) {
      return true;
    }
    return false;
  }
  
  // 获取模型的友好名称
  std::string getModelFriendlyName(const std::string& gazebo_name)
  {
    // 优先使用从model_state_publisher获取的友好名称
    auto fn_it = model_friendly_names_.find(gazebo_name);
    if (fn_it != model_friendly_names_.end()) {
      return fn_it->second;
    }
    
    // 其次使用本地映射
    auto it = model_name_map_.find(gazebo_name);
    if (it != model_name_map_.end()) {
      return it->second;
    }
    // 如果没有映射，尝试从名称推断
    if (gazebo_name.find("Ball") != std::string::npos) {
      return "球 (" + gazebo_name + ")";
    }
    if (gazebo_name.find("Table") != std::string::npos) {
      return "桌子 (" + gazebo_name + ")";
    }
    if (gazebo_name.find("Chair") != std::string::npos) {
      return "椅子 (" + gazebo_name + ")";
    }
    if (gazebo_name.find("Refrigerator") != std::string::npos) {
      return "冰箱 (" + gazebo_name + ")";
    }
    if (gazebo_name.find("Bed") != std::string::npos) {
      return "床 (" + gazebo_name + ")";
    }
    return gazebo_name;
  }
  
  // 模型状态回调 - 接收JSON格式的模型列表
  void modelStatesCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    Json::Value root;
    Json::CharReaderBuilder reader;
    std::string errs;
    std::istringstream stream(msg->data);
    
    if (!Json::parseFromStream(reader, stream, &root, &errs)) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Failed to parse model states JSON: %s", errs.c_str());
      return;
    }
    
    model_positions_.clear();
    model_friendly_names_.clear();
    
    if (root.isArray()) {
      for (const auto& model : root) {
        std::string name = model.get("name", "").asString();
        std::string friendly_name = model.get("friendly_name", "").asString();
        double x = model.get("x", 0.0).asDouble();
        double y = model.get("y", 0.0).asDouble();
        
        if (!shouldFilterModel(name)) {
          model_positions_[name] = {x, y};
          if (!friendly_name.empty()) {
            model_friendly_names_[name] = friendly_name;
          }
        }
      }
    }
    
    updateDynamicMapInfo();
    
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
      "Updated model states: %zu objects tracked", model_positions_.size());
  }
  
  // 更新动态地图信息
  void updateDynamicMapInfo()
  {
    std::stringstream ss;
    ss << "\n      【环境地图信息 (动态更新)】\n";
    ss << "      当前场景中检测到的物体及其坐标(x, y)：\n";
    
    std::vector<std::pair<std::string, std::pair<double, double>>> balls;
    std::vector<std::pair<std::string, std::pair<double, double>>> furniture;
    
    for (const auto& [name, pos] : model_positions_) {
      if (name.find("Ball") != std::string::npos) {
        balls.push_back({name, pos});
      } else {
        furniture.push_back({name, pos});
      }
    }
    
    if (!balls.empty()) {
      ss << "      球类物体:\n";
      for (const auto& [name, pos] : balls) {
        ss << "        - " << getModelFriendlyName(name) 
           << ": (" << std::fixed << std::setprecision(2) << pos.first 
           << ", " << pos.second << ")\n";
      }
    }
    
    if (!furniture.empty()) {
      ss << "      家具物体:\n";
      for (const auto& [name, pos] : furniture) {
        ss << "        - " << getModelFriendlyName(name)
           << ": (" << std::fixed << std::setprecision(2) << pos.first 
           << ", " << pos.second << ")\n";
      }
    }
    
    dynamic_map_info_ = ss.str();
  }
  
  // 获取完整的system prompt (基础 + 动态地图)
  std::string getFullSystemPrompt()
  {
    std::string full_prompt = base_system_prompt_;
    
    // 查找并替换地图信息部分
    size_t map_start = full_prompt.find("【环境地图信息");
    size_t map_end = full_prompt.find("【语义理解规则】");
    
    if (map_start != std::string::npos && map_end != std::string::npos && !dynamic_map_info_.empty()) {
      full_prompt = full_prompt.substr(0, map_start) + 
                    dynamic_map_info_.substr(7) +  // 去掉开头的换行
                    "\n      " + full_prompt.substr(map_end);
    }
    
    return full_prompt;
  }
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
      
      // 添加系统提示词 - 使用动态生成的prompt
      std::string system_prompt = getFullSystemPrompt();
      if (!system_prompt.empty()) {
        Json::Value system_msg;
        system_msg["role"] = "system";
        system_msg["content"] = system_prompt;
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
    
    // 打印当前已知的物体数量
    if (!model_positions_.empty()) {
      RCLCPP_INFO(this->get_logger(), "Current map has %zu tracked objects", model_positions_.size());
    } else {
      RCLCPP_WARN(this->get_logger(), "No dynamic map info available, using static config");
    }
    
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
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr model_states_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr navigation_command_pub_;
  
  std::string api_key_;
  std::string api_url_;
  std::string model_;
  std::string base_system_prompt_;
  double temperature_;
  int max_tokens_;
  
  // 动态地图相关
  std::map<std::string, std::pair<double, double>> model_positions_;  // 模型名 -> (x, y)
  std::map<std::string, std::string> model_name_map_;  // Gazebo名 -> 友好名(本地)
  std::map<std::string, std::string> model_friendly_names_;  // Gazebo名 -> 友好名(来自publisher)
  std::set<std::string> filter_models_;  // 需要过滤的模型
  std::string dynamic_map_info_;  // 动态生成的地图信息字符串
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LLMAnalyzer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
