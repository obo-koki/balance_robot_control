#include <sys/mman.h>
#include <unistd.h>
#include "BalanceRealController.h"

int BalanceRealController::pi;
int BalanceRealController::EN_R_A;
int BalanceRealController::EN_R_B;
int BalanceRealController::EN_L_A;
int BalanceRealController::EN_L_B;
int BalanceRealController::count_R;
int BalanceRealController::count_L;

void BalanceRealController::init(ros::NodeHandle nh, ros::NodeHandle pnh){
    node_ = nh;
    reset();
    loadConfig();
    set_driver(pnh);
    BalanceBaseController::setupFilters();
    BalanceBaseController::setupControllers();

    sub = node_.subscribe("bobble/start_cmd", 1,
                                &BalanceRealController::subscriberCallBack, this);
    sub_cmd_vel = node_.subscribe("/bobble/cmd_vel", 1,
                                &BalanceRealController::cmdVelCallback, this);

    sub_imu_sensor_ = node_.subscribe("/imu/data", 1, &BalanceRealController::imuCB, this);
    process_timer_ = node_.createWallTimer(ros::WallDuration(MAIN_PROCESS_PERIOD),&BalanceRealController::update, this);
    pub_timer_ = node_.createWallTimer(ros::WallDuration(PUBLISH_PERIOD), &BalanceRealController::publish, this);
    
    //dynamic param
    callback_ = boost::bind(&BalanceRealController::param_callback, this, _1, _2);
    param_server_.setCallback(callback_);
    ros::spin();
}

void BalanceRealController::set_driver(ros::NodeHandle pnh){
    // get param
    BalanceBaseController::unpackParameter("EN_R_A", EN_R_A, 23); //pnh.param<type>("param name", param_variable, default value);
    BalanceBaseController::unpackParameter("EN_R_B", EN_R_B, 24);
    BalanceBaseController::unpackParameter("EN_L_A", EN_L_A, 17);
    BalanceBaseController::unpackParameter("EN_L_B", EN_L_B, 27);

    BalanceBaseController::unpackParameter("PULSE_NUM", PULSE_NUM, 11);
    BalanceBaseController::unpackParameter("REDUCTION_RATIO", REDUCTION_RATIO, 90);

    BalanceBaseController::unpackParameter("MOTOR_DRIVER_RI1", MOTOR_DRIVER_RI1, 12);
    BalanceBaseController::unpackParameter("MOTOR_DRIVER_RI2", MOTOR_DRIVER_RI2, 25);

    BalanceBaseController::unpackParameter("MOTOR_DRIVER_LI1", MOTOR_DRIVER_LI1, 13);
    BalanceBaseController::unpackParameter("MOTOR_DRIVER_LI2", MOTOR_DRIVER_LI2, 26);

    BalanceBaseController::unpackParameter("MOTOR_FREQ", MOTOR_FREQ, 50000);

    BalanceBaseController::unpackParameter("PWM_RANGE", PWM_RANGE, 255);

    BalanceBaseController::unpackParameter("MAIN_PROCESS_PERIOD", MAIN_PROCESS_PERIOD, 0.002);
    BalanceBaseController::unpackParameter("PUBLISH_PERIOD", PUBLISH_PERIOD, 0.05);

    count_turn_en = 4 * PULSE_NUM;
    count_turn_out = count_turn_en * REDUCTION_RATIO;

    driver = new DRV8833();
    driver->setPinMode(pi, MOTOR_DRIVER_LI1, MOTOR_DRIVER_LI2, MOTOR_DRIVER_RI1, MOTOR_DRIVER_RI2);

    // pigpio
    pi = pigpio_start(0,0);
    if ( pi < 0 ){
        ROS_ERROR("pigpio Initialize Error (Forget $sudo pigpiod?)\n");
        exit(1);
    }

    //Wiring Pi
    //GPIO setup -> Encoder
    if(wiringPiSetupGpio() == -1){
        ROS_ERROR("wiringPi Initialize Error");
        exit(1);
    }

    pinMode(EN_R_A, INPUT);
    pinMode(EN_R_B, INPUT);
    wiringPiISR(EN_R_A, INT_EDGE_BOTH, &encoder_count_R_A);
    wiringPiISR(EN_R_B, INT_EDGE_BOTH, &encoder_count_R_B);

    pinMode(EN_L_A, INPUT);
    pinMode(EN_L_B, INPUT);
    wiringPiISR(EN_L_A, INT_EDGE_BOTH, &encoder_count_L_A);
    wiringPiISR(EN_L_B, INT_EDGE_BOTH, &encoder_count_L_B);

    //Initialize Encoder count
    count_R = 0;
    count_R_pre = 0;

    count_L = 0;
    count_L_pre = 0;

    //Initialize Odometry
    odom_x = 0.0; //[m]
    odom_y = 0.0; //[m]
    odom_th = 0.0; //[rad]
}

void BalanceRealController::loadConfig() {
    BalanceBaseController::unpackParameter("OutputToPwmFactor", config.OutputToPwmFactor, 800.0);
    BalanceBaseController::loadConfig();
}

void BalanceRealController::starting() {
    BalanceBaseController::reset();
    // Reset Madgwick Q on start is a sim only thing. The sim
    // resets the orientation on transition from Idle to Balance
    q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
}

void BalanceRealController::update(const ros::WallTimerEvent &e){
    mutex_.lock();
    /// Reset the quaternion every time we go to idle in sim
    if (state.ActiveControlMode == ControlModes::IDLE) {
        q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    }
    BalanceBaseController::update();
    mutex_.unlock();
}

void BalanceRealController::publish(const ros::WallTimerEvent &e){
    mutex_.lock();
    // Print for debug
    printf("robot_pitch:%3.2f\n",state.Tilt * 180.0 / M_PI);
    printf("robot_pitch_vel:%3.2f\n",state.TiltDot);
    printf("【Motor_R】count:%i,angle_vel_R:%3.2f,pwm_R:%3.2f\n", 
    count_R, state.MeasuredRightMotorVelocity,outputs.RightMotorEffortCmd);
    printf("【Motor_L】count:%i,angle_vel_L:%3.2f,pwm_L:%3.2f\n", 
    count_L, state.MeasuredLeftMotorVelocity, outputs.LeftMotorEffortCmd);
    printf("\n");

    // Odom pub
    //odom_pub_.publish(odom_);
    mutex_.unlock();
}

void BalanceRealController::imuCB(const sensor_msgs::Imu::ConstPtr &imuData) {
    state.MeasuredTiltDot = imuData->angular_velocity.y;
    state.MeasuredTurnRate = imuData->angular_velocity.z;
    state.MeasuredTilt = atan(-imuData->linear_acceleration.x/(sqrt(pow(imuData->linear_acceleration.y,2)+pow(imuData->linear_acceleration.z,2))));//+pitch_center_;
    //ROS_INFO("Subscribe Imu info");
    /*
    MadgwickAHRSupdateIMU(config.MadgwickFilterGain, imuData->angular_velocity.x,
                            imuData->angular_velocity.y, imuData->angular_velocity.z,
                            imuData->linear_acceleration.x, imuData->linear_acceleration.y,
                            imuData->linear_acceleration.z);
    tf::Quaternion q(q0, q1, q2, q3);
    tf::Matrix3x3 m(q);
    m.getRPY(state.MeasuredHeading, state.MeasuredTilt, state.MeasuredRoll);
    state.MeasuredTilt *= -1.0;
    */
}

void BalanceRealController::subscriberCallBack(const balance_robot_control::ControlCommands::ConstPtr &cmd) {
    received_commands.StartupCmd = cmd->StartupCmd;
    received_commands.IdleCmd = cmd->IdleCmd;
    received_commands.DiagnosticCmd = cmd->DiagnosticCmd;
}

void BalanceRealController::cmdVelCallback(const geometry_msgs::Twist& command) {
    //ROS_INFO("Subscribe cmd vel");
    received_commands.DesiredVelocity = command.linear.x;
    received_commands.DesiredTurnRate = command.angular.z;
}

void BalanceRealController::param_callback(const balance_robot_control::gain_bobbleConfig& gain, uint32_t level){
    ROS_INFO("Received");
    config.TiltControlKp = gain.TiltControlKp;
    pid_controllers.TiltControlPID.setP(gain.TiltControlKp);
    config.TiltControlKd = gain.TiltControlKd;
    pid_controllers.TiltControlPID.setD(gain.TiltControlKd, 0.0);
}

void BalanceRealController::estimateState(){
    // Position is not used by pendulum
    state.MeasuredLeftMotorPosition = 0.0;
    state.MeasuredRightMotorPosition = 0.0;
    // Measure motor vel from encoder
    state.MeasuredLeftMotorVelocity = (360.0 * (count_L - count_L_pre)/ count_turn_out / MAIN_PROCESS_PERIOD);
    count_L_pre = count_L;
    state.MeasuredRightMotorVelocity= (360.0 * (count_R - count_R_pre)/ count_turn_out / MAIN_PROCESS_PERIOD);
    count_R_pre = count_R;
}

void BalanceRealController::sendMotorCommands(){
    driver->drive(driver->A, outputs.LeftMotorEffortCmd * config.OutputToPwmFactor);
    driver->drive(driver->B, outputs.RightMotorEffortCmd * config.OutputToPwmFactor);
}

void BalanceRealController::encoder_count_R_A(){
    if (digitalRead(EN_R_A) == digitalRead(EN_R_B)){
        count_R --;
    }else{
        count_R ++;
    }
}

void BalanceRealController::encoder_count_R_B(){
    if (digitalRead(EN_R_A) != digitalRead(EN_R_B)){
        count_R --;
    }else{
        count_R ++;
    }
}

void BalanceRealController::encoder_count_L_A(){
    if (digitalRead(EN_L_A) == digitalRead(EN_L_B)){
        count_L ++;
    }else{
        count_L --;
    }
}

void BalanceRealController::encoder_count_L_B(){
    if (digitalRead(EN_L_A) != digitalRead(EN_L_B)){
        count_L ++;
    }else{
        count_L --;
    }
}