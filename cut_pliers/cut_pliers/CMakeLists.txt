cmake_minimum_required(VERSION 3.8)
project(cut_pliers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(serial REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(custom_msgs REQUIRED)

ament_package()

# Add executables
# 添加 test_node 可执行文件
add_executable(test_node src/test.cpp)

# 声明依赖关系
ament_target_dependencies(test_node
  rclcpp
  serial
  custom_msgs
  sensor_msgs
  message_filters
  geometry_msgs
  turtlesim
  tf2
  tf2_ros
)

add_executable(psub_and_RT src/psub_and_RT.cpp)
ament_target_dependencies(psub_and_RT 
  rclcpp 
  message_filters 
  serial 
  sensor_msgs 
  turtlesim 
  tf2 
  tf2_ros 
  geometry_msgs 
  custom_msgs)

add_executable(keyboard src/keyboard.cpp)
ament_target_dependencies(keyboard 
  rclcpp 
  sensor_msgs)

add_executable(sub_odom src/sub_odom.cpp)
ament_target_dependencies(sub_odom 
  rclcpp 
  message_filters 
  geometry_msgs 
  turtlesim 
  tf2 
  tf2_ros)

# Install executables
install(TARGETS 
  psub_and_RT
  keyboard
  sub_odom
  DESTINATION lib/${PROJECT_NAME})

# Install directories
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# 安装 test_node
install(TARGETS 
  test_node
  DESTINATION lib/${PROJECT_NAME})