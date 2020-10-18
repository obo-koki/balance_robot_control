# Balance Robot Control
倒立振子移動ロボットのコントロールプログラム

# Requirement

# Installation
imu

lidar

# Usage
imu(mpu6050)情報の表示
```
// c++
roslaunch balance_robot_control publish_imu_filtered_cpp.launch
// python
roslaunch balance_robot_control publish imu_filtered_py.launch
```
ロボットコントローラーの立ち上げ + ロボット情報の表示
```
roslaunch balance_robot_control balance_robot_control.launch
```
