cmake_minimum_required(VERSION 3.8)
project(tortoisebot_waypoints)

# Compiler options for better code quality
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wdeprecated -fPIC)
endif()

# Find package dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)

# Declare dependencies list for easier maintenance
set(dependencies
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  tf2_geometry_msgs
  custom_interfaces
)

# Define executable
add_executable(tortoisebot_action_server src/tortoisebot_action_server.cpp)
ament_target_dependencies(tortoisebot_action_server ${dependencies})

# Install executable
install(TARGETS
  tortoisebot_action_server
  DESTINATION lib/${PROJECT_NAME}
)

# Setup testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_${PROJECT_NAME} test/test_waypoint_action.cpp)
  ament_target_dependencies(test_${PROJECT_NAME} ${dependencies})
  
  install(TARGETS test_${PROJECT_NAME}
          DESTINATION lib/${PROJECT_NAME})
endif()

# Finalize package setup
ament_package()
