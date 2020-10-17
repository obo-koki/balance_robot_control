cmake_minimum_required(VERSION 3.0.2)
project(balance_robot_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#IMU driver node
add_executable(mpu6050 src/mpu6050.cpp)
target_link_libraries(mpu6050
  ${catkin_LIBRARIES}
  wiringPi
)

#add_executable(balance_robot_control src/balance_robot_control.cpp)
#target_link_libraries(balance_robot_control
  #${catkin_LIBRARIES}
  #wiringPi
#)
#add_executable(balance_robot_control src/balance_robot_control.cpp)
#target_link_libraries(balance_robot_control
  #${catkin_LIBRARIES}
  #wiringPi
#)