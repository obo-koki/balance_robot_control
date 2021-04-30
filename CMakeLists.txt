cmake_minimum_required(VERSION 3.0.2)
project(balance_robot_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  tf
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/gain.cfg
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#IMU driver
add_executable(mpu6050 src/mpu6050.cpp)
target_link_libraries(mpu6050
  ${catkin_LIBRARIES}
  wiringPi
)

#Motor Control
add_executable(balance_robot_control 
  src/control_main.cpp
  src/balance_robot_control.hpp
  src/balance_robot_control.cpp
  src/base_robot_control_drv8833.hpp
  src/base_robot_control_drv8833.cpp
  src/drv8833.hpp
)

target_link_libraries(balance_robot_control
  ${catkin_LIBRARIES}
  wiringPi
  pigpiod_if2
)