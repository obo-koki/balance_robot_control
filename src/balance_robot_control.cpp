#include "balance_robot_control.h"

BalanceRobotControl::BalanceRobotControl(){
    if(wiringPiSetupGpio() == -1){
        ROS_ERROR("wiringPi Initialize Error");
    }
    pinMode(EN_R_A, INPUT);
    pinMode(EN_R_B, INPUT);
    pinMode(EN_L_A, INPUT);
    pinMode(EN_L_B, INPUT);
    wiringPiISR(EN_R_A, INT_EDGE_BOTH, encoder_count_R_A);
    wiringPiISR(EN_R_B, INT_EDGE_BOTH, encoder_count_R_B);
    wiringPiISR(EN_L_A, INT_EDGE_BOTH, encoder_count_L_A);
    wiringPiISR(EN_L_B, INT_EDGE_BOTH, encoder_count_L_B);
}
  
void BalanceRobotControl::encoder_count_R_A(){
    if (digitalRead(EN_R_A) == digitalRead(EN_R_B)){
        encoder_count_R --;
    }else{
        encoder_count_R ++;
    }
}

void BalanceRobotControl::encoder_count_R_B(){
    if (digitalRead(EN_R_A) != digitalRead(EN_R_B)){
        encoder_count_R --;
    }else{
        encoder_count_R ++;
    }
}

void BalanceRobotControl::encoder_count_L_A(){
    if (digitalRead(EN_L_A) == digitalRead(EN_L_B)){
        encoder_count_L --;
    }else{
        encoder_count_L ++;
    }
}

void BalanceRobotControl::encoder_count_L_B(){
    if (digitalRead(EN_L_A) != digitalRead(EN_L_B)){
        encoder_count_L --;
    }else{
        encoder_count_L ++;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "balance_robot_control");
    ros::NodeHandle n;
  
    while (1)
    {
        delay(50);
    }
    return 0;
}