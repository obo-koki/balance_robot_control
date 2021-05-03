#include "balance_robot_control.hpp"

BalanceRobotControl::BalanceRobotControl(ros::NodeHandle nh, ros::NodeHandle pnh)
:BaseRobotControl_DRV8833(nh, pnh),robot_pitch_controlPID(0.0,0.0,0.0,0.0,0.0){
    pnh.getParam("control_gain", control_gain_);
    pnh.getParam("use_safe_mode", use_safe_mode_);
    pnh.getParam("use_run_mode", use_run_mode_);
    pnh.getParam("MadgwickFilterGain", MadgwickFilterGain_);

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
    wheel_angle_vel = 0;
    wheel_angle_vel_pre = 0;
    linear_x = 0;
    target_linear_x = 0;
    target_angular_z = 0;
    volt = 0;

    //Filter setting
    robot_pitch_filter.resetFilterParameters(1.0/MAIN_PROCESS_PERIOD, IMU_MEASURED_PERIOD, 1.0);
    robot_pitch_vel_filter.resetFilterParameters(1.0/MAIN_PROCESS_PERIOD,IMU_MEASURED_PERIOD,1.0);

    robot_pitch_controlPID.setPID(gain_theta_, 0.0, gain_omega_, 0.0, 1.0/MAIN_PROCESS_PERIOD);
    robot_pitch_controlPID.setExternalDerivativeError(&robot_pitch_vel);
    robot_pitch_controlPID.setOutputFilter(0.05); // output low path filter
    robot_pitch_controlPID.setOutputLimits(-PWM_RANGE,PWM_RANGE);
    robot_pitch_controlPID.setDirection(false);
    robot_pitch_controlPID.setSetpointRange(20.0 * (M_PI / 180.0));

    // Process period debug
    imu_sub_old = ros::Time::now();
    control_old = ros::Time::now();
}

void BalanceRobotControl::imu_callback(const sensor_msgs::Imu::ConstPtr &imu){
    std::lock_guard<std::mutex> lock(m);
    // low path filter
    //double measured_pitch = atan(-imu->linear_acceleration.x/(sqrt(pow(imu->linear_acceleration.y,2)+pow(imu->linear_acceleration.z,2))))+pitch_center_;
    //robot_pitch = robot_pitch_filter.filter(measured_pitch);
    
    double measured_pitch_vel = imu->angular_velocity.y - 3.13;
    robot_pitch_vel = robot_pitch_vel_filter.filter(measured_pitch_vel);

    //imu_sub_now = ros::Time::now();
    //ros::Duration time = imu_sub_now - imu_sub_old;
    //ROS_INFO("IMU sub time: %u.%09u",time.sec, time.nsec);
    //ROS_INFO("robot_pitch: %lf, robot_pitch_vel: %lf", measured_pitch, measured_pitch_vel);
    //imu_sub_old = imu_sub_now;

    MadgwickAHRSupdateIMU(MadgwickFilterGain_, imu->angular_velocity.x,
                            imu->angular_velocity.y, imu->angular_velocity.z,
                            imu->linear_acceleration.x, imu->linear_acceleration.y,
                            imu->linear_acceleration.z);
    
    tf::Quaternion q(q0, q1, q2, q3);
    tf::Matrix3x3 m(q);

    double measured_pitch, measured_row, measured_yaw;
    m.getRPY(measured_yaw, measured_pitch, measured_row);
    robot_pitch = robot_pitch_filter.filter(measured_pitch);
}

void BalanceRobotControl::vel_callback(const geometry_msgs::Twist::ConstPtr &vel){
    //get target velocity
    target_linear_x = vel->linear.x;
    target_angular_z = vel->angular.z;
}

void BalanceRobotControl::param_callback(const balance_robot_control::gainConfig& config, uint32_t level){
    ROS_INFO("Received dynamic param | gain_theta:%lf,gain_omega:%lf gain_fai:%lf gain_error:%lf",
        config.gain_theta, config.gain_omega, config.gain_fai, config.gain_error);
    
    gain_theta_ = config.gain_theta;
    robot_pitch_controlPID.setP(gain_theta_);
    gain_omega_ = config.gain_omega;
    robot_pitch_controlPID.setD(gain_omega_, 0.0);
    gain_fai_ = config.gain_fai;
    gain_error_ = config.gain_error;

    pitch_center_ = config.pitch_center;
    safe_radius_ = config.safe_radius;
}

void BalanceRobotControl::motor_control(){

    if (abs(robot_pitch) > 0.6){
        // give up mode -> No useless control
        pwm_L = 0.0;
        pwm_R = 0.0;

    }else if ((abs(robot_pitch) > safe_radius_ && use_safe_mode_)|| !use_run_mode_){
        // safe mode ->LQR pose control
        //wheel_angle_vel = (angle_vel_R + angle_vel_L) / 2.0;
        //volt = gain_theta_ * (robot_pitch)
                //+gain_omega_ * (robot_pitch_vel);
                //+gain_fai_ * (wheel_angle_vel);
        //      -gain_error_ * diff * 1.0/10.0;
        volt = robot_pitch_controlPID.getOutput(0,robot_pitch);

        //// Motor doesn't move range -50<pwm<50
        //if (volt > 0.01){
            //pwm_R = volt * 180/12 + 75;
        //}else if (volt < -0.01){
            //pwm_R = volt * 180/12 - 75;
        //}else{
            //pwm_R = 0;
        //}
        pwm_R = volt * 255/12;
        pwm_R = std::min(std::max(-1*PWM_RANGE,pwm_R),PWM_RANGE);

        //// Motor doesn't move range -50<pwm<50
        //if (volt > 0.01){
            //pwm_L = volt * 180/12 + 75;
        //}else if (volt < -0.01){
            //pwm_L = volt * 180/12 - 75;
        //}else{
            //pwm_L = 0;
        //}
        pwm_L = volt * 255/12;
        pwm_L = std::min(std::max(-1*PWM_RANGE,pwm_L),PWM_RANGE);

    }else if (use_run_mode_){
        // run mode ->PID velocity control 
        diff_R = target_vel_R - vel_R;
        integral_R += (diff_R + diff_pre_R)/2.0*MAIN_PROCESS_PERIOD;
        differential_R = (diff_R - diff_pre_R)/MAIN_PROCESS_PERIOD;

        pwm_R = KP_R*diff_R + KI_R*integral_R + KD_R*differential_R;
        pwm_R = std::min(std::max(-1*PWM_RANGE,pwm_R),PWM_RANGE);

        diff_L = target_vel_L - vel_L;
        integral_L += (diff_L + diff_pre_L)/2.0*MAIN_PROCESS_PERIOD;
        differential_L = (diff_L - diff_pre_L)/MAIN_PROCESS_PERIOD;

        pwm_L = KP_L*diff_L + KI_L*integral_L + KD_L*differential_L;
        pwm_L = std::min(std::max(-1*PWM_RANGE,pwm_L),PWM_RANGE);
    }
    driver->drive(driver->A, pwm_L);
    driver->drive(driver->B, pwm_R);

    control_now = ros::Time::now();
    ros::Duration time = control_now - control_old;
    //ROS_INFO("Control time: %u.%09u",time.sec, time.nsec);
    control_old = control_now;
}
void BalanceRobotControl::main_loop(){
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate rate(5); //[hz]
    ROS_INFO("Start Loop");
    while (ros::ok())
    {
        // Print for debug
        printf("robot_pitch:%3.2f\n",robot_pitch);
        printf("robot_pitch_vel:%3.2f\n",robot_pitch_vel);
        printf("wheel_angle_vel:%3.2f\n",wheel_angle_vel);
        printf("volt:%3.2f\n",volt);
        printf("【Motor_R】count:%i,angle_out:%3.2f,angle_vel_R:%3.2f,target_vel_R:%3.2f,vel_R:%3.2f,pwm_R:%d\n", 
        count_R, angle_out_R, angle_vel_R,target_vel_R,vel_R,pwm_R);
        printf("【Motor_L】count:%i,angle_out:%3.2f,angle_vel_L:%3.2f,target_vel_L:%3.2f,vel_L:%3.2f,pwm_L:%d\n\n",
        count_L, angle_out_L, angle_vel_L,target_vel_L,vel_L,pwm_L);

        // Odom pub
        odom_pub_.publish(odom_);
        rate.sleep();
    }
    motor_stop();
    spinner.stop();
}