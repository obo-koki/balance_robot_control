#include "balance_robot_control.hpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "balance_robot_control");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    BalanceRobotControl control(nh, pnh);
    control.main_loop();
    return 0;
}