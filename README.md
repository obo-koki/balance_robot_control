# Balance Robot Control
倒立振子移動ロボットのコントロールプログラム

# Requirement

# Installation
IMU

https://github.com/ccny-ros-pkg/imu_tools/tree/melodic

Lidar

https://github.com/Vidicon/camsense_driver
# Usage

ロボットコントローラーの立ち上げ
```
roslaunch balance_robot_control balance_robot_control.launch
```
IMU(mpu6050) 立ち上げ
```
// c++
roslaunch balance_robot_control publish_imu_filtered_cpp.launch
// python
roslaunch balance_robot_control publish_imu_filtered_py.launch
```
Lidar 立ち上げ
```
roslaunch balance_robot_control publish_lidar.launch
```
表示用 rviz 立ち上げ
```
roslaunch balance_robot_control display.launch
```
全部（IMU Lidar ロボットコントローラー）立ち上げ + 表示用 rviz
```
roslaunch balance_robot_control all_display.launch
```
