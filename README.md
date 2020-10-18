# Balance Robot Control
倒立振子移動ロボットのコントロールプログラム

# Usage
imu(mpu6050)情報の表示
```
// c++
roslaunch balance_robot_control publish_imu_filtered_cpp.launch
// python
roslaunch balance_robot_control publish imu_filtered_py.launch
```
ロボットコントローラーの立ち上げ
```
roslaunch balance_robot_control balance_robot_control.launch
```
