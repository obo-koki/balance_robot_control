# Balance Robot Control
倒立振子移動ロボットのコントロールプログラム

# Requirement
pigpiod -> raspiのGPIO操作用ライブラリ

wiringPi -> raspiのGPIO操作用ライブラリ
# Installation
IMU

https://github.com/ccny-ros-pkg/imu_tools/tree/melodic

Lidar

https://github.com/Vidicon/camsense_driver
# Usage
## 自作コントローラ
pigpiodの有効化（初期動作時のみ必要）

```
sudo pigpiod
```
ロボットコントローラー + IMUノードの立ち上げ
```
roslaunch balance_robot_control imu_control.launch
```
Lidar 立ち上げ
```
roslaunch balance_robot_control publish_lidar.launch
```
全部（ロボットコントローラー、IMU、Lidar）+ 表示用 rviz 立ち上げ
```
roslaunch balance_robot_control all_display.launch
```
動的パラメータ(dynamic_param)のGUIでの調整
```
rosrun rqt_reconfigure rqt_reconfigure
```

## 参考コントローラ(bobble)
ロボットコントローラー + IMUノードの立ち上げ
```
roslaunch balance_robot_control bobble_imu_control.launch
```

# Reference
シミュレーション用プログラム

https://github.com/obo-koki/balance_robot_description

参考プログラム(bobble_controllers)

https://github.com/obo-koki/bobble_controllers