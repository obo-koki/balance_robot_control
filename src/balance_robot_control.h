#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <cmath>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>

#include "imu.h"

// Pin Info
////Encoder
#define EN_R_A 23
#define EN_R_B 24
#define EN_L_A 23
#define EN_L_B 24
////IMU

class BalanceRobotControl{
    public:
        //variable
        static double encoder_count_R;
        static double encoder_count_L;
        //ros
        ros::NodeHandle node_handle_;
        ////Publisher
        ros::Publisher imu_pub_;
        ros::Publisher odom_pub_;
        ////Subscriber
        ros::Subscriber vel_sub_;
        ////Broadcaster for odom tf
        tf::TransformBroadcaster odom_broadcaster;
        //Constructor
        BalanceRobotControl();
        //Function
        static void encoder_count_R_A();
        static void encoder_count_R_B();
        static void encoder_count_L_A();
        static void encoder_count_L_B();

        void initialize_encoder();
};

double BalanceRobotControl::encoder_count_R;
double BalanceRobotControl::encoder_count_L;