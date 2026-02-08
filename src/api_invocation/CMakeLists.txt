cmake_minimum_required(VERSION 3.8)
project(api_invocation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(CURL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

# 1. llm_analyze executable
add_executable(llm_analyze src/llm_analyze.cpp)
target_include_directories(llm_analyze PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CURL_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS})
target_compile_features(llm_analyze PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  llm_analyze
  "rclcpp"
  "std_msgs"
)
target_link_libraries(llm_analyze
  ${CURL_LIBRARIES}
  ${JSONCPP_LIBRARIES}
)

# 2. test_publisher executable
add_executable(test_publisher src/test_publisher.cpp)
target_include_directories(test_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(test_publisher PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  test_publisher
  "rclcpp"
  "std_msgs"
)

# 3. simple_executor executable (for Task 1 simple demo)
add_executable(simple_executor src/simple_executor.cpp)
target_include_directories(simple_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${JSONCPP_INCLUDE_DIRS})
target_compile_features(simple_executor PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  simple_executor
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
)
target_link_libraries(simple_executor
  ${JSONCPP_LIBRARIES}
)

# 4. advanced_executor executable (支持语义导航视觉检查)
add_executable(advanced_executor src/advanced_executor.cpp)
target_include_directories(advanced_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${JSONCPP_INCLUDE_DIRS}
  ${CURL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})
target_compile_features(advanced_executor PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  advanced_executor
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "nav_msgs"
  "sensor_msgs"
  "tf2"
  "tf2_geometry_msgs"
)
target_link_libraries(advanced_executor
  ${JSONCPP_LIBRARIES}
  ${CURL_LIBRARIES}
  ${OpenCV_LIBS}
)

# 5. odom_to_tf executable (将 /odom 话题转换为 tf)
add_executable(odom_to_tf src/odom_to_tf.cpp)
target_include_directories(odom_to_tf PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(odom_to_tf PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  odom_to_tf
  "rclcpp"
  "nav_msgs"
  "tf2_ros"
  "geometry_msgs"
)

# 6. ground_truth_odom executable (将 Gazebo ground truth pose 转换为 odometry)
add_executable(ground_truth_odom src/ground_truth_odom.cpp)
target_include_directories(ground_truth_odom PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ground_truth_odom PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  ground_truth_odom
  "rclcpp"
  "nav_msgs"
  "geometry_msgs"
  "tf2"
)

# 7. voice_input executable (交互式自然语言命令输入终端 - 文字版)
add_executable(voice_input src/voice_input.cpp)
target_include_directories(voice_input PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(voice_input PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  voice_input
  "rclcpp"
  "std_msgs"
)

# 7.1 voice_input_asr executable (语音识别版本 - 使用麦克风录音 + 百度ASR)
# 需要安装: sudo apt install libasound2-dev
find_package(PkgConfig REQUIRED)
pkg_check_modules(ALSA alsa)
if(ALSA_FOUND)
  add_executable(voice_input_asr src/voice_input_asr.cpp)
  target_include_directories(voice_input_asr PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${CURL_INCLUDE_DIRS}
    ${JSONCPP_INCLUDE_DIRS}
    ${ALSA_INCLUDE_DIRS})
  target_compile_features(voice_input_asr PUBLIC c_std_99 cxx_std_17)
  ament_target_dependencies(
    voice_input_asr
    "rclcpp"
    "std_msgs"
  )
  target_link_libraries(voice_input_asr
    ${CURL_LIBRARIES}
    ${JSONCPP_LIBRARIES}
    ${ALSA_LIBRARIES}
  )
  install(TARGETS voice_input_asr
    DESTINATION lib/${PROJECT_NAME})
  message(STATUS "✓ voice_input_asr: 语音识别版本已启用 (ALSA found)")
else()
  message(WARNING "⚠ voice_input_asr: ALSA not found, 语音识别版本未编译。安装: sudo apt install libasound2-dev")
endif()

# 8. model_state_publisher executable (从Gazebo获取模型状态发布为JSON)
add_executable(model_state_publisher src/model_state_publisher.cpp)
target_include_directories(model_state_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${JSONCPP_INCLUDE_DIRS})
target_compile_features(model_state_publisher PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  model_state_publisher
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "tf2_msgs"
)
target_link_libraries(model_state_publisher
  ${JSONCPP_LIBRARIES}
)

# 9. image_viewer executable (相机图像查看器)
add_executable(image_viewer src/image_viewer.cpp)
target_include_directories(image_viewer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})
target_compile_features(image_viewer PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  image_viewer
  "rclcpp"
  "sensor_msgs"
)
target_link_libraries(image_viewer
  ${OpenCV_LIBS}
)

install(TARGETS llm_analyze test_publisher simple_executor advanced_executor odom_to_tf ground_truth_odom voice_input model_state_publisher image_viewer
  DESTINATION lib/${PROJECT_NAME})

# 安装 launch 文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 安装 config 文件
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()