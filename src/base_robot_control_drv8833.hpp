#ifndef BASE_ROBOT_CONTROL_DRV8833_HPP
#define BASE_ROBOT_CONTROL_DRV8833_HPP

#include <ros/ros.h>
#include <wiringPi.h>
#include <pigpiod_if2.h>
#include <stdio.h>
#include <chrono>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <mutex>
#include "drv8833.hpp"


class DRV8833;

#define PI 3.1415926535

class BaseRobotControl_DRV8833{
    protected:
    //Encoder
        static int EN_R_A; //Green
        static int EN_R_B; //Yellow

        static int EN_L_A; //Green
        static int EN_L_B; //Yellow

        int PULSE_NUM; 
        int REDUCTION_RATIO;

        float PROCESS_PERIOD; //[sec]

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

        // Timer callback debug
        ros::WallTime pre_time;
        ros::WallTime time; 
        std::mutex m;

        //Interrupt function -> wiringpi
        static void encoder_count_R_A();
        static void encoder_count_R_B();
        static void encoder_count_L_A();
        static void encoder_count_L_B();

        //ros
        nav_msgs::Odometry odom_;
        ros::Publisher odom_pub_;
        ros::Publisher vel_pub_R_; // for PID debug
        ros::Publisher vel_pub_L_; // for PID debug
        ros::Subscriber PID_sub_R_; // for PID debug
        void PID_R_callback(const std_msgs::Float32MultiArray&);
        ros::Subscriber PID_sub_L_; // for PID debug
        void PID_L_callback(const std_msgs::Float32MultiArray&);
        ros::Subscriber vel_sub_;
        virtual void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr&);
        ros::Subscriber imu_sub_;
        virtual void imu_callback(const sensor_msgs::Imu::ConstPtr&);
        ros::WallTimer process_timer_;
        //ros::Publisher imu_pub_;
        //tf::TransformBroadcaster odom_broadcaster;
        virtual void timer_callback(const ros::WallTimerEvent&);

        //Other
        virtual float calc_angle_output(int);
        virtual void calc_odom();
        virtual void motor_control();

    public:
        BaseRobotControl_DRV8833(ros::NodeHandle, ros::NodeHandle);
        virtual void motor_stop();
        virtual void main_loop();
        
};

#endif