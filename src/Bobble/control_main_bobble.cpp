#include "BalanceRealController.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "balance_robot_control");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    BalanceRealController control;
    control.init(nh, pnh);
    control.main_loop();
    return 0;
}