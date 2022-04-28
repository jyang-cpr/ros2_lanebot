cmake_minimum_required(VERSION 3.5)
project(ros2_lanebot)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(${PROJECT_NAME} src/node.cpp)
ament_target_dependencies(${PROJECT_NAME}
  cv_bridge
  OpenCV
  rclcpp
  sensor_msgs
)

set_target_properties(${PROJECT_NAME}
  PROPERTIES OUTPUT_NAME ${PROJECT_NAME} PREFIX "")

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()