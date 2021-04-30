#ifndef BALANCE_ROBOT_CONTROL_HPP
#define BALANCE_ROBOT_CONTROL_HPP

//#include "base_robot_control.hpp"
//#include "base_robot_control_tb.hpp"
#include "base_robot_control_drv8833.hpp"
#include <dynamic_reconfigure/server.h>
#include <balance_robot_control/gainConfig.h>

class BalanceRobotControl: public BaseRobotControl_DRV8833
{
    protected:
        void imu_callback(const sensor_msgs::Imu::ConstPtr&);
        void vel_callback(const geometry_msgs::Twist::ConstPtr&);
        void motor_control();

        double robot_pitch;
        double robot_pitch_pre;
        double robot_pitch_vel;
        double robot_pitch_vel_pre;
        double linear_x;
        double target_linear_x;
        double target_angular_z;
        double wheel_angle_vel;
        double wheel_angle_vel_pre;
        double diff;
        double diff_pre;
        double volt;
        std::vector<double> control_gain_forward_;
        std::vector<double> control_gain_backward_;

        //Dynamic param
        dynamic_reconfigure::Server<balance_robot_control::gainConfig> param_server_;
        dynamic_reconfigure::Server<balance_robot_control::gainConfig>::CallbackType callback_;
        void param_callback(const balance_robot_control::gainConfig& config, uint32_t level);

        double gain_theta_forward_;
        double gain_omega_forward_;
        double gain_fai_forward_;
        double gain_error_forward_;

        double gain_theta_backward_;
        double gain_omega_backward_;
        double gain_fai_backward_;
        double gain_error_backward_;

    public:
        //Other
        BalanceRobotControl(ros::NodeHandle, ros::NodeHandle);
        void main_loop();
};

#endif