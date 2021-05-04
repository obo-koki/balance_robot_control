#ifndef BOBBLE_CONTROLLERS_BALANCE_SIM_CONTROLLER_H
#define BOBBLE_CONTROLLERS_BALANCE_SIM_CONTROLLER_H

#include <cstddef>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <thread>
#include <mutex>

#include <sensor_msgs/Imu.h>
#include "BalanceBaseController.h"
#include <tf/transform_datatypes.h>
#include "../drv8833.hpp"
#include <wiringPi.h>
#include <pigpiod_if2.h>
#include <dynamic_reconfigure/server.h>
#include <balance_robot_control/gain_bobbleConfig.h>

class BalanceRealController : public BalanceBaseController
{
    public:
        BalanceRealController():BalanceBaseController(){};
        ~BalanceRealController(){};

        void init(ros::NodeHandle nh, ros::NodeHandle pnh);
        void set_driver(ros::NodeHandle pnh);
        void starting();
        void update(const ros::WallTimerEvent&);
        void publish(const ros::WallTimerEvent&);
        void motor_stop();
        void main_loop();


        //Interrupt function -> wiringpi
        static void encoder_count_R_A();
        static void encoder_count_R_B();
        static void encoder_count_L_A();
        static void encoder_count_L_B();

    protected:
        virtual void loadConfig();
        virtual void estimateState();
        virtual void sendMotorCommands();

        //Encoder
        static int EN_R_A; //Green
        static int EN_R_B; //Yellow

        static int EN_L_A; //Green
        static int EN_L_B; //Yellow

        int PULSE_NUM; 
        int REDUCTION_RATIO;

        //Process period
        float MAIN_PROCESS_PERIOD; //[sec]
        float PUBLISH_PERIOD;

        //Motor Driver DRV8833
        int MOTOR_DRIVER_RI1; //Forward pwm
        int MOTOR_DRIVER_RI2; //Backward pwm

        int MOTOR_DRIVER_LI1; //Forward pwm
        int MOTOR_DRIVER_LI2; //Backward pwm

        //PWM
        int PWM_RANGE; 
        int MOTOR_FREQ; 

        //Odometry
        float WHEEL_DIA; //[m]
        float WHEEL_DIST; //[m]

        //Others
        static int pi;
        DRV8833* driver;

        //Encoder
        int count_turn_en;
        int count_turn_out;

        static int count_R;
        int count_R_pre;

        static int count_L;
        int count_L_pre;

        // Odometry
        float odom_x; //[m]
        float odom_y; //[m]
        float odom_th; //[rad]

        ros::WallTimer process_timer_;
        ros::WallTimer pub_timer_;

        //Dynamic param
        std::vector<double> control_gain_;
        double pitch_center_;
        double safe_radius_;
        dynamic_reconfigure::Server<balance_robot_control::gain_bobbleConfig> param_server_;
        dynamic_reconfigure::Server<balance_robot_control::gain_bobbleConfig>::CallbackType callback_;
        void param_callback(const balance_robot_control::gain_bobbleConfig& config, uint32_t level);

    private:
        ros::Subscriber sub_imu_sensor_;
        ros::Subscriber sub;
        ros::Subscriber sub_cmd_vel;
        void imuCB(const sensor_msgs::Imu::ConstPtr &imuData);
        void subscriberCallBack(const balance_robot_control::ControlCommands::ConstPtr &cmd);
        void cmdVelCallback(const geometry_msgs::Twist& command);
};
#endif