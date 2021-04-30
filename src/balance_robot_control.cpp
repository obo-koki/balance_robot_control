#include "balance_robot_control.hpp"

BalanceRobotControl::BalanceRobotControl(ros::NodeHandle nh, ros::NodeHandle pnh)
:BaseRobotControl_DRV8833(nh){
    pnh.getParam("control_gain_forward", control_gain_forward_);
    pnh.getParam("control_gain_backward", control_gain_backward_);

    //dynamic param
    callback_ = boost::bind(&BalanceRobotControl::param_callback, this, _1, _2);
    param_server_.setCallback(callback_);
    //Timer callback debug
    pre_time = ros::WallTime::now();

    //Initialize
    robot_pitch = 0;
    robot_pitch_pre = 0;
    robot_pitch_vel = 0;
    robot_pitch_vel_pre = 0;
    linear_x = 0;
    target_linear_x = 0;
    target_angular_z = 0;
    volt = 0;
}

void BalanceRobotControl::imu_callback(const sensor_msgs::Imu::ConstPtr &imu){
    std::lock_guard<std::mutex> lock(m);
    robot_pitch = atan(-imu->linear_acceleration.x/(sqrt(pow(imu->linear_acceleration.y,2)+pow(imu->linear_acceleration.z,2))))+0.015;
    robot_pitch_vel = imu->angular_velocity.y + 3.13;
}

void BalanceRobotControl::vel_callback(const geometry_msgs::Twist::ConstPtr &vel){
    //get target velocity
    target_linear_x = vel->linear.x;
    target_angular_z = vel->angular.z;
}

void BalanceRobotControl::param_callback(const balance_robot_control::gainConfig& config, uint32_t level){
    ROS_INFO("Received dynamic param | gain_theta_forward:%lf,gain_omega_forward:%lf gain_fai_forward:%lf gain_error_forward:%lf",
        config.gain_theta_forward, config.gain_omega_forward, config.gain_fai_forward, config.gain_error_forward);
    ROS_INFO("Received dynamic param | gain_theta_backward:%lf,gain_omega_backward:%lf gain_fai_backward:%lf gain_error_backward:%lf",
        config.gain_theta_backward, config.gain_omega_backward, config.gain_fai_backward, config.gain_error_backward);
    
    gain_theta_forward_ = config.gain_theta_forward;
    gain_omega_forward_ = config.gain_omega_forward;
    gain_fai_forward_ = config.gain_fai_forward;
    gain_error_forward_ = config.gain_error_forward;

    gain_theta_backward_ = config.gain_theta_backward;
    gain_omega_backward_ = config.gain_omega_backward;
    gain_fai_backward_ = config.gain_fai_backward;
    gain_error_backward_ = config.gain_error_backward;
}

void BalanceRobotControl::motor_control(){

    if (abs(robot_pitch) > 0.5){
        //give up
        volt = 0.00;
    } else {
        if(robot_pitch >= 0) {
        wheel_angle_vel = (angle_vel_R + angle_vel_L) / 2.0;
        volt = gain_theta_backward_ * (robot_pitch)
                +gain_omega_backward_ * (robot_pitch_vel);
                +gain_fai_backward_ * (wheel_angle_vel);
        //      -gain_error_backward_ * diff * 1.0/10.0;
        } else {
            wheel_angle_vel = (angle_vel_R + angle_vel_L) / 2.0;
            volt = gain_theta_forward_ * (robot_pitch)
                    +gain_omega_forward_ * (robot_pitch_vel);
                    +gain_fai_forward_ * (wheel_angle_vel);
            //      -gain_error_forward_ * diff * 1.0/10.0;
        }
    }

    // PID
    //diff_R = target_vel_R - vel_R;
    //integral_R += (diff_R + diff_pre_R)/2.0*PROCESS_PERIOD;
    //differential_R = (diff_R - diff_pre_R)/PROCESS_PERIOD;

    //pwm_R = KP_R*diff_R + KI_R*integral_R + KD_R*differential_R;
    //pwm_R = std::min(std::max(-1*PWM_RANGE,pwm_R),PWM_RANGE);

    //diff_L = target_vel_L - vel_L;
    //integral_L += (diff_L + diff_pre_L)/2.0*PROCESS_PERIOD;
    //differential_L = (diff_L - diff_pre_L)/PROCESS_PERIOD;

    //pwm_L = KP_L*diff_L + KI_L*integral_L + KD_L*differential_L;
    //pwm_L = std::min(std::max(-1*PWM_RANGE,pwm_L),PWM_RANGE);

    // Motor doesn't move range -30<pwm<30
    if (volt > 0.01){
        pwm_R = volt * 225/12 + 50;
    }else if (volt < -0.01){
        pwm_R = volt * 225/12 - 50;
    }else{
        pwm_R = 0;
    }
    pwm_R = std::min(std::max(-1*PWM_RANGE,pwm_R),PWM_RANGE);

    // Motor doesn't move range -30<pwm<30
    if (volt > 0.01){
        pwm_L = volt * 225/12 + 50;
    }else if (volt < -0.01){
        pwm_L = volt * 225/12 - 50;
    }else{
        pwm_L = 0;
    }
    pwm_L = std::min(std::max(-1*PWM_RANGE,pwm_L),PWM_RANGE);

    //printf("pwm_R:%d, pwm_L:%d", pwm_R, pwm_L);

    driver->drive(driver->A, pwm_L);
    driver->drive(driver->B, pwm_R);
}
void BalanceRobotControl::main_loop(){
    //Motor start
    //gpio_write(pi, MOTOR_DRIVER_EN, PI_LOW);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate rate(5);
    ROS_INFO("Start Loop");
    while (ros::ok())
    {
        printf("robot_pitch:%3.2f\n",robot_pitch);
        printf("robot_pitch_vel:%3.2f\n",robot_pitch_vel);
        printf("wheel_angle_vel:%3.2f\n",wheel_angle_vel);
        printf("volt:%3.2f\n",volt);
        printf("【Motor_R】count:%i,angle_out:%3.2f,angle_vel_R:%3.2f,target_vel_R:%3.2f,vel_R:%3.2f,pwm_R:%d\n", 
        count_R, angle_out_R, angle_vel_R,target_vel_R,vel_R,pwm_R);
        printf("【Motor_L】count:%i,angle_out:%3.2f,angle_vel_L:%3.2f,target_vel_L:%3.2f,vel_L:%3.2f,pwm_L:%d\n\n",
        count_L, angle_out_L, angle_vel_L,target_vel_L,vel_L,pwm_L);
        odom_pub_.publish(odom_);
        rate.sleep();
    }
    motor_stop();
    spinner.stop();
}