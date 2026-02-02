cmake_minimum_required(VERSION 3.8)
project(ackermann_v2)

# 找依赖
find_package(ament_cmake REQUIRED)

# 安装 launch/model/config 文件夹
install(
  DIRECTORY launch model config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
