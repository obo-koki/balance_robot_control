#ifndef TB6612_H
#define TB6612_H

// Copy from https://github.com/botamochi6277/tamiya_cam_robot.git

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <pigpiod_if2.h>

class TB6612 {
 private:
  
 public:
  int pi_; // pigpio
  // Motor A
  int pin_ain1_;
  int pin_ain2_;
  int pin_apwm_;
  // Motor B
  int pin_bin1_;
  int pin_bin2_;
  int pin_bpwm_;

  static const bool A = 0;
  static const bool B = 1;
  TB6612(){}
  TB6612(int pi,
         int pin_ain1,
         int pin_ain2,
         int pin_apwm,
         int pin_bin1,
         int pin_bin2,
         int pin_bpwm
        ) {
    pi_                 = pi;
    pin_ain1_           = pin_ain1;
    pin_ain2_           = pin_ain2;
    pin_apwm_           = pin_apwm;
    pin_bin1_           = pin_bin1;
    pin_bin2_           = pin_bin2;
    pin_bpwm_           = pin_bpwm;

    set_mode(pi_, pin_ain1_, PI_OUTPUT);
    set_mode(pi_, pin_ain2_, PI_OUTPUT);
    set_mode(pi_, pin_apwm_, PI_OUTPUT);
    set_mode(pi_, pin_bin1_, PI_OUTPUT);
    set_mode(pi_, pin_bin2_, PI_OUTPUT);
    set_mode(pi_, pin_bpwm_, PI_OUTPUT);

  }


  // Set Pin mode
  void setPinMode(int pi,
         int pin_ain1,
         int pin_ain2,
         int pin_apwm,
         int pin_bin1,
         int pin_bin2,
         int pin_bpwm)
  {
    TB6612::pin_ain1_ = 2;
    //ROS_INFO("pi %d", pi_);
    //ROS_INFO("ain %d",pin_ain1_);
    
    pi_                 = pi;
    pin_ain1_           = pin_ain1;
    pin_ain2_           = pin_ain2;
    pin_apwm_           = pin_apwm;
    pin_bin1_           = pin_bin1;
    pin_bin2_           = pin_bin2;
    pin_bpwm_           = pin_bpwm;
    
    
    set_mode(pi_, pin_ain1_, PI_OUTPUT);
    set_mode(pi_, pin_ain2_, PI_OUTPUT);
    set_mode(pi_, pin_apwm_, PI_OUTPUT);
    set_mode(pi_, pin_bin1_, PI_OUTPUT);
    set_mode(pi_, pin_bin2_, PI_OUTPUT);
    set_mode(pi_, pin_bpwm_, PI_OUTPUT);
    
    ROS_INFO("Set TB6612 Pin mode");
  }

  /**
   * drive a motor
   * @param motor motor's id
   * @param power power of a motor (-255 -- 255)
   */
  void drive(int motor, int power) {
    switch (motor) {
    case A :
      // Motor-A
      if (power > 0) {
        gpio_write(pi_, pin_ain1_, 1);
        gpio_write(pi_, pin_ain2_, 0);
      } else {
        gpio_write(pi_, pin_ain1_, 0);
        gpio_write(pi_, pin_ain2_, 1);
        power = -power;
      }
      set_PWM_dutycycle(pi_, pin_apwm_, power);
      break;

    case B:
      // Motor-B
      if (power > 0) {
        gpio_write(pi_, pin_bin1_, 1);
        gpio_write(pi_, pin_bin2_, 0);
      } else {
        gpio_write(pi_, pin_bin1_, 0);
        gpio_write(pi_, pin_bin2_, 1);
        power = -power;
      }
      set_PWM_dutycycle(pi_, pin_bpwm_, power);

      break;
    }
  }

  void serialCallback(const std_msgs::String::ConstPtr& msg) {
    static int pwm0, pwm1;
    static char rot0, rot1;
    std::sscanf(msg->data.c_str(), "M0%c%03dM1%c%03d", &rot0, &pwm0, &rot1, &pwm1);

    ROS_INFO("I heard: [M0%c%03d, M1%c%03d]", rot0, pwm0, rot1, pwm1);

    if (rot0 == 'R') {
      pwm0 *= -1;
    }

    if (rot1 == 'R') {
      pwm1 *= -1;
    }
    drive(TB6612::A, pwm0);
    drive(TB6612::B, pwm1);

  }

 
  void twistCallback(const geometry_msgs::Twist::ConstPtr& msg){
    static int pwm_r, pwm_l;
    pwm_r = 255*(msg->linear.x + msg->angular.z);
    pwm_l = 255*(msg->linear.x - msg->angular.z);

    // limit duty ratio 
    if (std::abs(pwm_r) > 255){
      float c = 255.0/std::abs(pwm_r);
      pwm_r*= c;
      pwm_l*= c;
    }

    if (std::abs(pwm_l) > 255){
      float c = 255.0/std::abs(pwm_l);
      pwm_r*= c;
      pwm_l*= c;
    }
    drive(TB6612::A, pwm_r);
    drive(TB6612::B, pwm_l);

    // ROS_INFO("motor: %d,%d",pwm_r,pwm_l);
  }


};
#endif