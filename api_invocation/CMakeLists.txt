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
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(CURL REQUIRED)
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

# 4. advanced_executor executable (for Task 1, 2, 3 - High Level)
add_executable(advanced_executor src/advanced_executor.cpp)
target_include_directories(advanced_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${JSONCPP_INCLUDE_DIRS})
target_compile_features(advanced_executor PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  advanced_executor
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "nav_msgs"
  "tf2"
  "tf2_geometry_msgs"
)
target_link_libraries(advanced_executor
  ${JSONCPP_LIBRARIES}
)

install(TARGETS llm_analyze test_publisher simple_executor advanced_executor
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()