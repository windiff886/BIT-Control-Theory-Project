/**
 * @file model_state_publisher.cpp
 * @brief 从Gazebo获取所有模型的位置信息，并以JSON格式发布
 * 
 * 方案1: 订阅 Gazebo 的 /world/<world_name>/pose/info 话题 (通过ros_gz_bridge)
 * 方案2: 解析world文件获取静态模型初始位置
 * 
 * 以 JSON 格式发布到 /gazebo/model_states 供 LLM 分析器使用
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <json/json.h>
#include <fstream>
#include <regex>
#include <map>
#include <set>

class ModelStatePublisher : public rclcpp::Node
{
public:
  ModelStatePublisher() : Node("model_state_publisher")
  {
    // 声明参数
    this->declare_parameter("world_file", "");
    world_file_ = this->get_parameter("world_file").as_string();
    
    // 发布模型状态 (JSON格式)
    model_states_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/gazebo/model_states", 10);
    
    // 订阅 pose topic (如果通过bridge可用)
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/world/house/pose/info", 10,
      std::bind(&ModelStatePublisher::poseCallback, this, std::placeholders::_1));
    
    // 订阅 TF 作为备选
    tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", 10,
      std::bind(&ModelStatePublisher::tfCallback, this, std::placeholders::_1));
    
    // 定时发布 (2Hz)
    publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&ModelStatePublisher::publishModelStates, this));
    
    // 初始化过滤器
    initFilters();
    
    // 尝试从world文件解析静态位置
    if (!world_file_.empty()) {
      parseWorldFile(world_file_);
    } else {
      // 使用默认路径尝试
      parseDefaultWorld();
    }
    
    RCLCPP_INFO(this->get_logger(), "Model State Publisher initialized");
    RCLCPP_INFO(this->get_logger(), "Initial models loaded: %zu", model_positions_.size());
  }

private:
  void initFilters()
  {
    // 模型名映射
    model_name_map_["Ball_01_001"] = "红色球1 (Ball 1)";
    model_name_map_["Ball_01_002"] = "红色球2 (Ball 2)";
    model_name_map_["Ball_01_003"] = "红色球3 (Ball 3)";
    model_name_map_["Bed_01"] = "床 (Bed)";
    model_name_map_["NightStand_01_001"] = "床头柜1";
    model_name_map_["NightStand_01_002"] = "床头柜2";
    model_name_map_["CoffeeTable_01"] = "茶几 (Coffee Table)";
    model_name_map_["KitchenTable_01"] = "餐桌 (Kitchen Table)";
    model_name_map_["Refrigerator_01"] = "冰箱 (Refrigerator)";
    model_name_map_["KitchenCabinet_01"] = "橱柜 (Kitchen Cabinet)";
    model_name_map_["ChairA_01"] = "椅子A";
    model_name_map_["ChairD_01"] = "餐椅";
    model_name_map_["Dumbbell_01"] = "哑铃";
    model_name_map_["FitnessEquipment_01"] = "健身器材";
    model_name_map_["Pillow_01"] = "枕头";
    model_name_map_["CookingBench_01"] = "厨房台面";
    model_name_map_["BalconyTable_01"] = "阳台桌";
    
    // 需要过滤的关键词
    filter_keywords_.insert("Wall");
    filter_keywords_.insert("Floor");
    filter_keywords_.insert("Window");
    filter_keywords_.insert("Light");
    filter_keywords_.insert("Chandelier");
    filter_keywords_.insert("Portrait");
    filter_keywords_.insert("Handle");
    filter_keywords_.insert("Carpet");
    filter_keywords_.insert("ground");
    filter_keywords_.insert("sun");
    filter_keywords_.insert("Door");
    filter_keywords_.insert("Curtain");
    filter_keywords_.insert("ackermann");
    filter_keywords_.insert("Board");
    filter_keywords_.insert("Utensils");
    filter_keywords_.insert("Airconditioner");
  }
  
  bool shouldFilter(const std::string& name)
  {
    for (const auto& keyword : filter_keywords_) {
      if (name.find(keyword) != std::string::npos) {
        return true;
      }
    }
    return false;
  }
  
  void parseDefaultWorld()
  {
    // 尝试常见路径
    std::vector<std::string> paths = {
      "/home/tony/Desktop/BIT-control/src/br2_gazebo_worlds/worlds/house.world",
      "src/br2_gazebo_worlds/worlds/house.world"
    };
    
    for (const auto& path : paths) {
      if (parseWorldFile(path)) {
        RCLCPP_INFO(this->get_logger(), "Loaded world file: %s", path.c_str());
        return;
      }
    }
    RCLCPP_WARN(this->get_logger(), "Could not find world file, using dynamic updates only");
  }
  
  bool parseWorldFile(const std::string& filepath)
  {
    std::ifstream file(filepath);
    if (!file.is_open()) {
      return false;
    }
    
    std::string content((std::istreambuf_iterator<char>(file)),
                        std::istreambuf_iterator<char>());
    file.close();
    
    // 使用正则表达式解析 <include> 块
    // 格式: <include>...<name>XXX</name>...<pose>x y z r p y</pose>...</include>
    std::regex include_regex(R"(<include[^>]*>[\s\S]*?<name>([^<]+)</name>[\s\S]*?<pose>([^<]+)</pose>[\s\S]*?</include>)");
    
    auto begin = std::sregex_iterator(content.begin(), content.end(), include_regex);
    auto end = std::sregex_iterator();
    
    for (auto it = begin; it != end; ++it) {
      std::smatch match = *it;
      std::string name = match[1].str();
      std::string pose_str = match[2].str();
      
      if (shouldFilter(name)) continue;
      
      // 解析pose: "x y z roll pitch yaw"
      std::istringstream pose_stream(pose_str);
      double x, y, z;
      pose_stream >> x >> y >> z;
      
      if (!pose_stream.fail()) {
        model_positions_[name] = {x, y};
        RCLCPP_DEBUG(this->get_logger(), "Loaded model: %s at (%.2f, %.2f)", 
                     name.c_str(), x, y);
      }
    }
    
    return !model_positions_.empty();
  }
  
  void poseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    // 如果收到pose话题，使用它更新位置
    // 注意：PoseArray不包含名称，需要通过其他方式关联
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Received pose array with %zu poses", msg->poses.size());
  }
  
  void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    // 从TF更新动态物体位置
    for (const auto& transform : msg->transforms) {
      std::string frame_id = transform.child_frame_id;
      
      // 只处理从world出发的变换
      if (transform.header.frame_id != "world") {
        continue;
      }
      
      if (shouldFilter(frame_id)) continue;
      
      double x = transform.transform.translation.x;
      double y = transform.transform.translation.y;
      
      // 更新位置
      model_positions_[frame_id] = {x, y};
    }
  }
  
  std::string getFriendlyName(const std::string& gazebo_name)
  {
    auto it = model_name_map_.find(gazebo_name);
    if (it != model_name_map_.end()) {
      return it->second;
    }
    
    // 尝试模糊匹配
    for (const auto& [key, value] : model_name_map_) {
      if (gazebo_name.find(key) != std::string::npos) {
        return value;
      }
    }
    
    // 返回原名
    return gazebo_name;
  }
  
  void publishModelStates()
  {
    if (model_positions_.empty()) {
      return;
    }
    
    // 构建JSON数组
    Json::Value root(Json::arrayValue);
    
    for (const auto& [name, pos] : model_positions_) {
      Json::Value model;
      model["name"] = name;
      model["friendly_name"] = getFriendlyName(name);
      model["x"] = pos.first;
      model["y"] = pos.second;
      root.append(model);
    }
    
    // 转为字符串
    Json::StreamWriterBuilder writer;
    writer["indentation"] = "";
    std::string json_str = Json::writeString(writer, root);
    
    // 发布
    auto msg = std_msgs::msg::String();
    msg.data = json_str;
    model_states_pub_->publish(msg);
    
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Published %zu model states", model_positions_.size());
  }
  
  // 订阅器和发布器
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr model_states_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  
  // 数据
  std::string world_file_;
  std::map<std::string, std::pair<double, double>> model_positions_;
  std::map<std::string, std::string> model_name_map_;
  std::set<std::string> filter_keywords_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ModelStatePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
