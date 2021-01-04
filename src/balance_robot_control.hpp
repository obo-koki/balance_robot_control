#ifndef BALANCE_ROBOT_CONTROL_HPP
#define BALANCE_ROBOT_CONTROL_HPP

//#include "base_robot_control.hpp"
#include "base_robot_control_tb.hpp"

class BalanceRobotControl: public BaseRobotControl_TB
{
    public:
        //Other
        BalanceRobotControl(ros::NodeHandle);
};

#endif