#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include "mpu6050.h"
  
#define EN_A 23
#define EN_B 24
#define MOTOR_DIR_1 25
#define MOTOR_PWM_1 12
#define MOTOR_DRIVER_EN 5
#define MOTOR_DRIVER_FAULT 6

#define range 1024
#define PI 3.1415926535

int count = 0;

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

void stop_motor(){
    digitalWrite(MOTOR_DRIVER_EN,1);
    printf("Abnoromal motor condition\n");
    exit(1);
}
  
int main() {
    printf("init\n");
  
    if(wiringPiSetupGpio() == -1) return 1;

    //Encoder
    pinMode(EN_A, INPUT);
    pinMode(EN_B, INPUT);
    wiringPiISR(EN_A, INT_EDGE_BOTH, encoder_count_A);
    wiringPiISR(EN_B, INT_EDGE_BOTH, encoder_count_B);
    //Motor
    pinMode(MOTOR_DRIVER_EN, OUTPUT); // 0-> motor start, 1-> motor stop
    pinMode(MOTOR_DIR_1, OUTPUT);
    pinMode(MOTOR_PWM_1,PWM_OUTPUT);
    pwmSetMode(0);
    pwmSetRange(1024);
    //wiringPiISR(MOTOR_DRIVER_FAULT, INT_EDGE_RISING, stop_motor); // 0-> motor over-temperture or over-current, 1-> normally 
    digitalWrite(MOTOR_DIR_1, 0);

    imu_data imu;
  
    while (1)
    {
        imu = get_imu_data();
        printf("imu_yaw:%f\n",imu.yaw);
        digitalWrite(MOTOR_DRIVER_EN,0);
        pwmWrite(MOTOR_PWM_1,abs(imu.yaw)*200);
        if (imu.yaw >= 0){
            digitalWrite(MOTOR_DIR_1,0);
        }else{
            digitalWrite(MOTOR_DIR_1,1);
        }
        delay(500);
    }
    return 0;
}