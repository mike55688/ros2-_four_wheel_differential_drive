cmake_minimum_required(VERSION 3.8)
project(forklift_demo)

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

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

add_executable(psub_and_RT src/psub_and_RT.cpp)
ament_target_dependencies(psub_and_RT rclcpp message_filters serial sensor_msgs turtlesim tf2 tf2_ros geometry_msgs custom_msgs) 

add_executable(keyboard src/keyboard.cpp)
ament_target_dependencies(keyboard rclcpp sensor_msgs)

add_executable(sub_odom src/sub_odom.cpp)
ament_target_dependencies(sub_odom rclcpp message_filters geometry_msgs turtlesim tf2 tf2_ros )

install(TARGETS sub_odom
  DESTINATION lib/${PROJECT_NAME})
    
install(TARGETS psub_and_RT
  DESTINATION lib/${PROJECT_NAME}) 
    
install(TARGETS keyboard
  DESTINATION lib/${PROJECT_NAME})

  
  
