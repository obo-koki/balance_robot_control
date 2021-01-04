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
#include "tb6612.hpp"

//Encoder
#define EN_R_A 23 //Green
#define EN_R_B 24 //Yellow

#define EN_L_A 17 //Green
#define EN_L_B 27 //Yellow

#define PULSE_NUM 11 //encoder pulse number
#define REDUCTION_RATIO 90

#define PROCESS_PERIOD 0.001 //[sec]

//Motor Driver
//#define MOTOR_DIR_R 25
//#define MOTOR_PWM_R 12

//#define MOTOR_DIR_L 22
//#define MOTOR_PWM_L 13

//#define MOTOR_DRIVER_EN 5
//#define MOTOR_DRIVER_FAULT 6

//Motor Driver TB
#define MOTOR_DRIVER_RI1 22
#define MOTOR_DRIVER_RI2 25
#define MOTOR_PWM_R 12

#define MOTOR_DRIVER_LI1 5
#define MOTOR_DRIVER_LI2 6
#define MOTOR_PWM_L 13

//PWM
#define PWM_RANGE 255 // You can change 25~40000 (default 255)
#define MOTOR_FREQ 50000 // 50 kHz (= max motor driver freq)

//Odometry
#define WHEEL_DIA 0.066 //[m]
#define WHEEL_DIST 0.180 //[m]

//PID Gain
/*
#define KP_R 2.0
#define KI_R 1.0
#define KD_R 1.0

#define KP_L 2.0
#define KI_L 1.0
#define KD_L 1.0
*/

//Others
#define PI 3.1415926535

class TB6612;

class BalanceRobotControl{
    public:
        static int pi;
        
        //Motor Driver
        /**
        int pin_ain1 = MOTOR_DRIVER_RI1;
        int pin_ain2 = MOTOR_DRIVER_RI2;
        int pin_apwm = MOTOR_PWM_R;
        int pin_bin1 = MOTOR_DRIVER_LI1;
        int pin_bin2 = MOTOR_DRIVER_LI2;
        int pin_bpwm = MOTOR_PWM_L;
        */

        TB6612* driver;

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
        float target_vel_R; //[m/s]
        int pwm_R; // 0 ~ PWM_RANGE

        float vel_L; //[m/s]
        float target_vel_L; //[m/s]
        int pwm_L; // 0 ~ PWM_RANGE

        // Odometry
        float odom_x; //[m]
        float odom_y; //[m]
        float odom_th; //[rad]

        // PID
        float KP_R = 2.0;
        float KI_R = 1.0;
        float KD_R = 1.0;

        float KP_L = 2.0;
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

        //Interrupt function -> pigpio
        static void encoder_count_R_A(int, unsigned int, unsigned int, unsigned int);
        static void encoder_count_R_B(int, unsigned int, unsigned int, unsigned int);
        static void encoder_count_L_A(int, unsigned int, unsigned int, unsigned int);
        static void encoder_count_L_B(int, unsigned int, unsigned int, unsigned int);

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
        void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr&);
        ros::Subscriber imu_sub_;
        void imu_callback(const sensor_msgs::Imu::ConstPtr&);
        ros::WallTimer process_timer_;
        //ros::Publisher imu_pub_;
        //tf::TransformBroadcaster odom_broadcaster;
        void timer_callback(const ros::WallTimerEvent&);

        //Other
        BalanceRobotControl(ros::NodeHandle);
        float calc_angle_output(int);
        void calc_odom();
        void motor_stop();
        void motor_control();
        void main_loop();

        
};

int BalanceRobotControl::pi;
int BalanceRobotControl::count_R;
int BalanceRobotControl::count_L;