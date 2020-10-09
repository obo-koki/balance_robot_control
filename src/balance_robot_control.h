#include <wiringPi.h>
#include <stdio.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

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
        double encoder_count_R;
        double encoder_count_L;
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
        void encoder_count_R_A();
        void encoder_count_R_B();
        void encoder_count_L_A();
        void encoder_count_L_B();
};