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

#Encoder
add_executable(encoder src/encoder_counter.cpp)
target_link_libraries(encoder
  ${catkin_LIBRARIES}
  wiringPi
)

#Motor Control
#add_executable(balance_robot_control 
#  src/control_main.cpp
#  src/balance_robot_control.hpp
#  src/balance_robot_control.cpp
#  src/base_robot_control.hpp
#  src/base_robot_control.cpp
#)
add_executable(balance_robot_control 
  src/control_main.cpp
  src/balance_robot_control.hpp
  src/balance_robot_control.cpp
  src/base_robot_control_tb.hpp
  src/base_robot_control_tb.cpp
  src/tb6612.hpp
)

target_link_libraries(balance_robot_control
  ${catkin_LIBRARIES}
  wiringPi
  pigpiod_if2
)

#add_executable(balance_robot_control_tb src/balance_robot_control_tb.cpp)
#target_link_libraries(balance_robot_control_tb
#  ${catkin_LIBRARIES}
#  wiringPi
#  pigpiod_if2
#)