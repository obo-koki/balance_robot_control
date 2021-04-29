#ifndef DRV8833_HPP
#define DRV8833_HPP

// This is a driver for aliexpress drv8833 similar to tb6612

#include <pigpiod_if2.h>

class DRV8833 {
 private:
  
 public:
  int pi_; // pigpio
  // Motor A (L)
  int pin_ain1_; //Forward pwm
  int pin_ain2_; //Backward pwm
  // Motor B (R)
  int pin_bin1_; //Forward pwm 
  int pin_bin2_; //Backward pwm

  static const bool A = 0;
  static const bool B = 1;
  DRV8833(){}
  DRV8833(int pi,
         int pin_ain1,
         int pin_ain2,
         int pin_bin1,
         int pin_bin2
        ) {
    pi_                 = pi;
    pin_ain1_           = pin_ain1;
    pin_ain2_           = pin_ain2;
    pin_bin1_           = pin_bin1;
    pin_bin2_           = pin_bin2;

    set_mode(pi_, pin_ain1_, PI_OUTPUT);
    set_mode(pi_, pin_ain2_, PI_OUTPUT);
    set_mode(pi_, pin_bin1_, PI_OUTPUT);
    set_mode(pi_, pin_bin2_, PI_OUTPUT);
  }


  // Set Pin mode
  void setPinMode(int pi,
         int pin_ain1,
         int pin_ain2,
         int pin_bin1,
         int pin_bin2)
  {
    DRV8833::pin_ain1_ = 2;
    
    pi_                 = pi;
    pin_ain1_           = pin_ain1;
    pin_ain2_           = pin_ain2;
    pin_bin1_           = pin_bin1;
    pin_bin2_           = pin_bin2;

    set_mode(pi_, pin_ain1_, PI_OUTPUT);
    set_mode(pi_, pin_ain2_, PI_OUTPUT);
    set_mode(pi_, pin_bin1_, PI_OUTPUT);
    set_mode(pi_, pin_bin2_, PI_OUTPUT);
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
        set_PWM_dutycycle(pi_, pin_ain1_, power);
        set_PWM_dutycycle(pi_, pin_ain2_, 0);
      } else {
        power = -power;
        set_PWM_dutycycle(pi_, pin_ain1_, 0);
        set_PWM_dutycycle(pi_, pin_ain2_, power);
      }
      break;

    case B:
      // Motor-B
      if (power > 0) {
        set_PWM_dutycycle(pi_, pin_bin1_, power);
        set_PWM_dutycycle(pi_, pin_bin2_, 0);
      } else {
        power = -power;
        set_PWM_dutycycle(pi_, pin_bin1_, 0);
        set_PWM_dutycycle(pi_, pin_bin2_, power);
      }
      break;
    }
  }
};
#endif