#include <ros/ros.h>
#include <wiringPi.h>
#include <stdio.h>
#include <chrono>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#define EN_R_A 23 //Green
#define EN_R_B 24 //Yellow

#define EN_L_A 17 //Green
#define EN_L_B 27 //Yellow

#define PULSE_NUM 11 //encoder pulse number
#define REDUCTION_RATIO 90

#define PROCESS_PERIOD 0.001 //[sec]

const int count_turn_en = 4 * PULSE_NUM;
const int count_turn_out = count_turn_en * REDUCTION_RATIO;

int count_R = 0;
float angle_out_pre_R = 0.0; //[deg]
float angle_out_R = 0.0; //[deg]
float speed_R = 0.0; //[deg/s]

int count_L = 0;
float angle_out_pre_L = 0.0; //[deg]
float angle_out_L = 0.0; //[deg]
float speed_L = 0.0; //[deg/s]

void encoder_count_R_A(){
    int i,j;
    i = digitalRead(EN_R_A);
    j = digitalRead(EN_R_B);
    if (i == j){
        count_R --;
    }else{
        count_R ++;
    }
}
void encoder_count_R_B(){
    int i,j;
    i = digitalRead(EN_R_A);
    j = digitalRead(EN_R_B);
    if (i == j){
        count_R ++;
    }else{
        count_R --;
    }
}
void encoder_count_L_A(){
    int i,j;
    i = digitalRead(EN_L_A);
    j = digitalRead(EN_L_B);
    if (i == j){
        count_L --;
    }else{
        count_L ++;
    }
}
void encoder_count_L_B(){
    int i,j;
    i = digitalRead(EN_L_A);
    j = digitalRead(EN_L_B);
    if (i == j){
        count_L ++;
    }else{
        count_L --;
    }
}

float calc_angle_output(int _count){
    int count_temp = 360.0 / (count_turn_out) * (_count % (count_turn_out));
    if (count_temp >= 0)
        return count_temp;
    else
        return count_temp + 360.0;
}

void timer_callback(const ros::WallTimerEvent& e){
    //Motor R
    angle_out_R = calc_angle_output(count_R);
    speed_R = (angle_out_R - angle_out_pre_R) / PROCESS_PERIOD;
    angle_out_pre_R = angle_out_R;
    //Motor L
    angle_out_L = calc_angle_output(count_L);
    speed_L = (angle_out_L - angle_out_pre_L) / PROCESS_PERIOD;
    angle_out_pre_L = angle_out_L;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr &imu){
    printf("Received Imu info\n");
}

void cmd_callback(const geometry_msgs::Twist::ConstPtr &imu){
    printf("Received cmd_vel\n");
}

int main(int argc,char** argv) {

    if(wiringPiSetupGpio() == -1) return 1;
    //Motor R
    pinMode(EN_R_A, INPUT);
    pinMode(EN_R_B, INPUT);
    wiringPiISR(EN_R_A, INT_EDGE_BOTH, encoder_count_R_A);
    wiringPiISR(EN_R_B, INT_EDGE_BOTH, encoder_count_R_B);
    //Motor L
    pinMode(EN_L_A, INPUT);
    pinMode(EN_L_B, INPUT);
    wiringPiISR(EN_L_A, INT_EDGE_BOTH, encoder_count_L_A);
    wiringPiISR(EN_L_B, INT_EDGE_BOTH, encoder_count_L_B);

    ros::init(argc, argv, "encoder");
    ros::NodeHandle nh;

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom",1000);
    nav_msgs::Odometry odom;

    ros::Subscriber imu_sub = nh.subscribe("/imu/data", 10, imu_callback);
    ros::Subscriber cmd_sub = nh.subscribe("/cmd_vel", 10, cmd_callback);

    ros::WallTimer walltimer = nh.createWallTimer(ros::WallDuration(PROCESS_PERIOD), timer_callback);

    ros::Rate rate(5);
    while (ros::ok())
    {
        printf("Motor_R:count %i,angle_out %3.1f,speed %3.1f \n", count_R, angle_out_R, speed_R);
        printf("Motor_L:count %i,angle_out %3.1f,speed %3.1f \n\n", count_L, angle_out_L, speed_L);
        // cal_odom(odom);
        odom_pub.publish(odom);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}