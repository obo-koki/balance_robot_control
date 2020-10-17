#include <ros/ros.h>
#include <wiringPi.h>
#include <stdio.h>
#include <chrono>

#define EN_A 23 //Green
#define EN_B 24 //Yellow

#define PULSE_NUM 11 //encoder pulse number
#define REDUCTION_RATIO 90

#define PROCESS_PERIOD 0.001 //[sec]

const int count_turn_en = 4 * PULSE_NUM;
const int count_turn_out = count_turn_en * REDUCTION_RATIO;

int count = 0;

float angle_out_pre = 0.0; //[deg]
float angle_out = 0.0; //[deg]
float speed = 0.0; //[deg/s]


void encoder_count_A(){
    int i,j;
    i = digitalRead(EN_A);
    j = digitalRead(EN_B);
    if (i == j){
        count --;
    }else{
        count ++;
    }
}

void encoder_count_B(){
    int i,j;
    i = digitalRead(EN_A);
    j = digitalRead(EN_B);
    if (i == j){
        count ++;
    }else{
        count --;
    }
}

float calc_angle_encoder(int _count){
    int count_temp = 360.0 / (count_turn_en) * (_count % (count_turn_en));
    if (count_temp >= 0)
        return count_temp;
    else
        return count_temp + 360.0;
}

float calc_angle_output(int _count){
    int count_temp = 360.0 / (count_turn_out) * (_count % (count_turn_out));
    if (count_temp >= 0)
        return count_temp;
    else
        return count_temp + 360.0;
}

void timer_callback(const ros::WallTimerEvent& e){
    angle_out = calc_angle_output(count);
    speed = (angle_out - angle_out_pre) / PROCESS_PERIOD;
    angle_out_pre = angle_out;
}

int main(int argc,char** argv) {
    int i;

    if(wiringPiSetupGpio() == -1) return 1;

    pinMode(EN_A, INPUT);
    pinMode(EN_B, INPUT);
    wiringPiISR(EN_A, INT_EDGE_BOTH, encoder_count_A);
    wiringPiISR(EN_B, INT_EDGE_BOTH, encoder_count_B);

    ros::init(argc, argv, "encoder");
    ros::NodeHandle nh;

    ros::WallTimer walltimer = nh.createWallTimer(ros::WallDuration(PROCESS_PERIOD), timer_callback);

    ros::Rate rate(5);
    while (ros::ok())
    {
        printf("count %i, angle_out %3.1f,speed %3.1f \n", count, angle_out, speed);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}