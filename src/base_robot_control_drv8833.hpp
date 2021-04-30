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

//Encoder
#define EN_R_A 23 //Green
#define EN_R_B 24 //Yellow

#define EN_L_A 17 //Green
#define EN_L_B 27 //Yellow

#define PULSE_NUM 11 //encoder pulse number
#define REDUCTION_RATIO 90

#define PROCESS_PERIOD 0.0005 //[sec]

//Motor Driver DRV8833
#define MOTOR_DRIVER_RI1 12 //Forward pwm
#define MOTOR_DRIVER_RI2 25 //Backward pwm

#define MOTOR_DRIVER_LI1 13 //Forward pwm
#define MOTOR_DRIVER_LI2 26 //Backward pwm

//PWM
#define PWM_RANGE 255 // You can change 25~40000 (default 255)
#define MOTOR_FREQ 50000 // 50 kHz (= max motor driver freq)

//Odometry
#define WHEEL_DIA 0.066 //[m]
#define WHEEL_DIST 0.180 //[m]

//Others
#define PI 3.1415926535

class DRV8833;

class BaseRobotControl_DRV8833{
    protected:
        static int pi;
        
        DRV8833* driver;

        //Encoder
        const int count_turn_en = 4 * PULSE_NUM;
        const int count_turn_out = count_turn_en * REDUCTION_RATIO;

        static int count_R;
        int count_R_pre;
        float angle_out_R; //[deg]
        float angle_vel_R; //[deg/s]

        static int count_L;
        int count_L_pre;
        float angle_out_pre_L; //[deg]
        float angle_out_L; //[deg]
        float angle_vel_L; //[deg/s]

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
        float KP_R = 3.0;
        float KI_R = 1.0;
        float KD_R = 1.0;

        float KP_L = 3.0;
        float KI_L = 1.0;
        float KD_L = 1.0;

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
        ros::NodeHandle node_handle_;
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
        BaseRobotControl_DRV8833(ros::NodeHandle);
        virtual void motor_stop();
        virtual void main_loop();
        
};

#endif