#ifndef BALANCE_ROBOT_CONTROL_HPP
#define BALANCE_ROBOT_CONTROL_HPP

//#include "base_robot_control.hpp"
//#include "base_robot_control_tb.hpp"
#include "base_robot_control_drv8833.hpp"
#include <dynamic_reconfigure/server.h>
#include <balance_robot_control/gainConfig.h>
#include "Filter/LowPassFilter.h"
#include "Filter/PidControl.h"
#include "IMU/MadgwickAHRS.h"

class BalanceRobotControl: public BaseRobotControl_DRV8833
{
    protected:
        void imu_callback(const sensor_msgs::Imu::ConstPtr&);
        void vel_callback(const geometry_msgs::Twist::ConstPtr&);
        void motor_control();

        // Varible for state feed back
        double robot_pitch; //[rad]
        double robot_pitch_pre; //[rad]
        double robot_pitch_vel; //[rad/s]
        double robot_pitch_vel_pre; //[rad/s]
        double linear_x;
        double target_linear_x;
        double target_angular_z;
        double wheel_angle_vel;
        double wheel_angle_vel_pre;
        double diff;
        double diff_pre;
        double volt;

        // Process period debug
        ros::Time imu_sub_now;
        ros::Time imu_sub_old;
        ros::Time control_now;
        ros::Time control_old;

        //Dynamic param
        std::vector<double> control_gain_;
        double pitch_center_;
        double safe_radius_;
        dynamic_reconfigure::Server<balance_robot_control::gainConfig> param_server_;
        dynamic_reconfigure::Server<balance_robot_control::gainConfig>::CallbackType callback_;
        void param_callback(const balance_robot_control::gainConfig& config, uint32_t level);

        double gain_theta_;
        double gain_omega_;
        double gain_fai_;
        double gain_error_;

        // mode use flag
        bool use_safe_mode_;
        bool use_run_mode_;

        // filter
        LowPassFilter robot_pitch_filter;
        LowPassFilter robot_pitch_vel_filter;
        PidControl robot_pitch_controlPID;

        double MadgwickFilterGain_;

    public:
        //Other
        BalanceRobotControl(ros::NodeHandle, ros::NodeHandle);
        void main_loop();
};

#endif