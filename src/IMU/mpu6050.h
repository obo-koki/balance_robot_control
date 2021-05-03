#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <cmath>

using namespace std;

#define DEV_ADDR 0x68    // I2C
#define ACCEL_X_OUT 0x3b
#define ACCEL_Y_OUT 0x3d
#define ACCEL_Z_OUT 0x3f
#define TEMP_OUT 0x41
#define GYRO_X_OUT 0x43 
#define GYRO_Y_OUT 0x45
#define GYRO_Z_OUT 0x47
#define PWR_MGMT_1 0x6b  //PWR_MGMT_1
#define PWR_MGMT_2 0x6c  //PWR_MGMT_2
#define LPF_ADDR 0x1a

#define RAD_TO_DEG 57.324

typedef struct {
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float roll;
    float pitch;
    float yaw;
} imu_data;

int fd = wiringPiI2CSetup(DEV_ADDR);

float read_word_2c(int addr) {
  int high = wiringPiI2CReadReg8(fd, addr);
  int low = wiringPiI2CReadReg8(fd, addr+1);
  int val = (high << 8) + low;
  return float((val >= 0x8000) ? -((65535 - val) + 1) : val);
}

void calcEuler(float x, float y, float z, float *theta, float *psi, float *phi){
  *theta = atan(x / sqrt(y * y + z * z));
  *psi = atan(y / sqrt(x*x + z*z));
  *phi = atan(sqrt(x * x + y * y) / z);
}

void initialize_imu(){
  // Device starts in sleep mode so wake it up.
  wiringPiI2CWriteReg16(fd, PWR_MGMT_1, 0);
  // Set LRF
  wiringPiI2CWriteReg16(fd, LPF_ADDR, 0x03);
}

imu_data get_imu_data(){
    float ax,ay,az,gx,gy,gz,roll,pitch,yaw;
    printf("fd:%d\n",fd);
    ax = read_word_2c(ACCEL_X_OUT) / 16384.0;
    ay = read_word_2c(ACCEL_Y_OUT) / 16384.0;
    az = read_word_2c(ACCEL_Z_OUT) / 16384.0;
    gx = read_word_2c(GYRO_X_OUT) / 131.0;
    gy = read_word_2c(GYRO_Y_OUT) / 131.0;
    gz = read_word_2c(GYRO_Z_OUT) / 131.0;
    calcEuler(ax, ay, az, &roll, &pitch, &yaw);
    return (imu_data){ax,ay,az,gx,gy,gz,roll,pitch,yaw};
}