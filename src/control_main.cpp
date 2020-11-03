#include "balance_robot_control.hpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "balance_robot_control");
    ros::NodeHandle nh;
    BalanceRobotControl control(nh);
    control.main_loop();
    return 0;
}