#include "base_robot_control.hpp"

class BalanceRobotControl: public BaseRobotControl
{
    protected:
        virtual void motor_control();
    public:
        //Other
        BalanceRobotControl(ros::NodeHandle);
};