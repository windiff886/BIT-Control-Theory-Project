cmake_minimum_required(VERSION 3.8)
project(api_invocation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(CURL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

add_executable(llm_analyze src/llm_analyze.cpp)
target_include_directories(llm_analyze PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CURL_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS})
target_compile_features(llm_analyze PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  llm_analyze
  "rclcpp"
  "std_msgs"
)
target_link_libraries(llm_analyze
  ${CURL_LIBRARIES}
  ${JSONCPP_LIBRARIES}
)

# test_publisher executable
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

install(TARGETS llm_analyze test_publisher
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
