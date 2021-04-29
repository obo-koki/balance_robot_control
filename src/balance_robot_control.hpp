#ifndef BALANCE_ROBOT_CONTROL_HPP
#define BALANCE_ROBOT_CONTROL_HPP

//#include "base_robot_control.hpp"
//#include "base_robot_control_tb.hpp"
#include "base_robot_control_drv8833.hpp"

class BalanceRobotControl: public BaseRobotControl_DRV8833
{
    //protected:
        //virtual void motor_control();
    public:
        //Other
        BalanceRobotControl(ros::NodeHandle);
};

#endif