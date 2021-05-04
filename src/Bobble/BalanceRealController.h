/******************************************************************************
 * Gazebo ROS interface for Bobble-Bot's balance controller.
 *
*******************************************************************************/

#ifndef BOBBLE_CONTROLLERS_BALANCE_SIM_CONTROLLER_H
#define BOBBLE_CONTROLLERS_BALANCE_SIM_CONTROLLER_H

#include <cstddef>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <thread>
#include <mutex>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include "BalanceBaseController.h"
#include <tf/transform_datatypes.h>
#include "../drv8833.hpp"
#include <wiringPi.h>
#include <pigpiod_if2.h>

class BalanceRealController : public BalanceBaseController
{
    public:
        BalanceRealController(){};
        ~BalanceRealController(){};

        void init(ros::NodeHandle nh, ros::NodeHandle pnh);
        void set_driver(ros::NodeHandle pnh);
        void starting();
        void update(const ros::WallTimerEvent&);
        void motor_stop();
        void main_loop();

        //Interrupt function -> wiringpi
        static void encoder_count_R_A();
        static void encoder_count_R_B();
        static void encoder_count_L_A();
        static void encoder_count_L_B();

    protected:
        virtual void loadConfig();
        virtual void estimateState();
        virtual void sendMotorCommands();

        //Encoder
        static int EN_R_A; //Green
        static int EN_R_B; //Yellow

        static int EN_L_A; //Green
        static int EN_L_B; //Yellow

        int PULSE_NUM; 
        int REDUCTION_RATIO;

        //Process period
        float MAIN_PROCESS_PERIOD; //[sec]
        float IMU_MEASURED_PERIOD; //[sec]
        float VEL_MEASURED_PERIOD; //[sec]

        //Motor Driver DRV8833
        int MOTOR_DRIVER_RI1; //Forward pwm
        int MOTOR_DRIVER_RI2; //Backward pwm

        int MOTOR_DRIVER_LI1; //Forward pwm
        int MOTOR_DRIVER_LI2; //Backward pwm

        //PWM
        int PWM_RANGE; 
        int MOTOR_FREQ; 

        //Odometry
        float WHEEL_DIA; //[m]
        float WHEEL_DIST; //[m]

        //Others
        static int pi;
        DRV8833* driver;

        //Encoder
        int count_turn_en;
        int count_turn_out;

        static int count_R;
        int count_R_pre;
        float angle_out_R; //[deg]
        float angle_vel_R; //[deg/s]
        float angle_vel_R_pre; //[deg/s]

        static int count_L;
        int count_L_pre;
        float angle_out_pre_L; //[deg]
        float angle_out_L; //[deg]
        float angle_vel_L; //[deg/s]
        float angle_vel_L_pre; //[deg/s]

        // Wheel velocity
        float vel_R; //[m/s]
        float vel_R_pre; //[m/s]
        float vel_R_fil; //[m/s]
        float target_vel_R; //[m/s]
        int pwm_R; // 0 ~ PWM_RANGE

        float vel_L; //[m/s]
        float vel_L_pre; //[m/s]
        float vel_L_fil; //[m/s]
        float target_vel_L; //[m/s]
        int pwm_L; // 0 ~ PWM_RANGE

        // Vel filter param
        const float a_vel = 0.5;

        // Odometry
        float odom_x; //[m]
        float odom_y; //[m]
        float odom_th; //[rad]

        // PID
        float KP_R;
        float KI_R;
        float KD_R;

        float KP_L;
        float KI_L;
        float KD_L;

        float diff_R, diff_pre_R;
        float integral_R, differential_R;

        float diff_L, diff_pre_L;
        float integral_L, differential_L;

        ros::WallTimer process_timer_;

    private:
        ros::Subscriber sub_imu_sensor_;
        void imuCB(const sensor_msgs::Imu::ConstPtr &imuData);
};
#endif