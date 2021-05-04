#include <sys/mman.h>
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
    pub_bobble_status_ = new realtime_tools::RealtimePublisher<balance_robot_control::BobbleBotStatus>(node_,
                                                "bobble_balance_controller/bb_controller_status", 1);
    run_thread_ = true;
    subscriber_thread_ = new std::thread(&BalanceBaseController::runSubscriber, this);
    sub_imu_sensor_ = node_.subscribe("/imu/data", 1, &BalanceRealController::imuCB, this);
    process_timer_ = node_.createWallTimer(ros::WallDuration(MAIN_PROCESS_PERIOD),&BalanceRealController::update, this);
}

void BalanceRealController::set_driver(ros::NodeHandle pnh){
    // get param
    pnh.param<int>("EN_R_A", EN_R_A, 23); //pnh.param<type>("param name", param_variable, default value);
    pnh.param<int>("EN_R_B", EN_R_B, 24);
    pnh.param<int>("EN_L_A", EN_L_A, 17);
    pnh.param<int>("EN_L_B", EN_L_B, 27);

    pnh.param<int>("PULSE_NUM", PULSE_NUM, 11);
    pnh.param<int>("REDUCTION_RATIO", REDUCTION_RATIO, 90);

    pnh.param<int>("MOTOR_DRIVER_RI1", MOTOR_DRIVER_RI1, 12);
    pnh.param<int>("MOTOR_DRIVER_RI2", MOTOR_DRIVER_RI2, 25);

    pnh.param<int>("MOTOR_DRIVER_LI1", MOTOR_DRIVER_LI1, 13);
    pnh.param<int>("MOTOR_DRIVER_LI2", MOTOR_DRIVER_LI2, 26);

    pnh.param<int>("MOTOR_FREQ", MOTOR_FREQ, 50000);

    pnh.param<int>("PWM_RANGE", PWM_RANGE, 255);

    pnh.param<float>("KP_R", KP_R, 3.0);
    pnh.param<float>("KI_R", KI_R, 1.0);
    pnh.param<float>("KD_R", KD_R, 1.0);

    pnh.param<float>("KP_L", KP_L, 3.0);
    pnh.param<float>("KI_L", KI_L, 1.0);
    pnh.param<float>("KD_L", KD_L, 1.0);

    pnh.param<float>("WHEEL_DIA", WHEEL_DIA, 0.066);
    pnh.param<float>("WHEEL_DIST", WHEEL_DIST, 0.180);

    pnh.param<float>("MAIN_PROCESS_PERIOD", MAIN_PROCESS_PERIOD, 0.0005);
    pnh.param<float>("IMU_MEASURED_PERIOD", IMU_MEASURED_PERIOD, 0.0005);
    pnh.param<float>("VEL_MEASURED_PERIOD", VEL_MEASURED_PERIOD, 0.0005);

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
    angle_out_R = 0.0; //[deg]
    angle_vel_R = 0.0; //[deg/s]

    count_L = 0;
    count_L_pre = 0;
    angle_out_L = 0.0; //[deg]
    angle_vel_L = 0.0; //[deg/s]

    //Initialize Wheel velocity
    vel_R = vel_R_pre = vel_R_fil = 0.0; //[m/s]
    target_vel_R = 0.0; //[m/s]
    pwm_R = 0.0; // 0 ~ PWM_RANGE

    vel_L = vel_L_pre = vel_L_fil = 0.0; //[m/s]
    target_vel_L = 0.0; //[m/s]
    pwm_L = 0.0; // 0 ~ PWM_RANGE
    
    //Initialize Odometry
    odom_x = 0.0; //[m]
    odom_y = 0.0; //[m]
    odom_th = 0.0; //[rad]
}

void BalanceRealController::loadConfig() {
    BalanceBaseController::loadConfig();
}

void BalanceRealController::starting() {
    BalanceBaseController::reset();
    // Reset Madgwick Q on start is a sim only thing. The sim
    // resets the orientation on transition from Idle to Balance
    q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
}

void BalanceRealController::update(const ros::WallTimerEvent &e){
    /// Reset the quaternion every time we go to idle in sim
    if (state.ActiveControlMode == ControlModes::IDLE) {
        q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    }
    BalanceBaseController::update();
}

void BalanceRealController::imuCB(const sensor_msgs::Imu::ConstPtr &imuData) {
    state.MeasuredTiltDot = imuData->angular_velocity.y;
    state.MeasuredTurnRate = imuData->angular_velocity.z;
    MadgwickAHRSupdateIMU(config.MadgwickFilterGain, imuData->angular_velocity.x,
                            imuData->angular_velocity.y, imuData->angular_velocity.z,
                            imuData->linear_acceleration.x, imuData->linear_acceleration.y,
                            imuData->linear_acceleration.z);
    tf::Quaternion q(q0, q1, q2, q3);
    tf::Matrix3x3 m(q);
    m.getRPY(state.MeasuredHeading, state.MeasuredTilt, state.MeasuredRoll);
    state.MeasuredTilt *= -1.0;
}

void BalanceRealController::estimateState(){
    // Position is not used by pendulum
    state.MeasuredLeftMotorPosition = 0.0;
    state.MeasuredRightMotorPosition = 0.0;
    // Measure motor vel from encoder
    angle_vel_L = (360.0 * (count_L - count_L_pre)/ count_turn_out / MAIN_PROCESS_PERIOD);
    count_L_pre = count_L;
    angle_vel_R = (360.0 * (count_R - count_R_pre)/ count_turn_out / MAIN_PROCESS_PERIOD);
    count_R_pre = count_R;
    state.MeasuredLeftMotorVelocity = angle_vel_L;
    state.MeasuredRightMotorVelocity = angle_vel_R;
}

void BalanceRealController::sendMotorCommands(){
    driver->drive(driver->A, outputs.LeftMotorEffortCmd);
    driver->drive(driver->B, outputs.RightMotorEffortCmd);
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

void BalanceRealController::main_loop(){
    ros::Rate rate(5);
    while (ros::ok())
    {
        // Print for debug
        printf("robot_pitch:%3.2f\n",state.Tilt * 180.0 / M_PI);
        printf("robot_pitch_vel:%3.2f\n",state.TiltDot);
        printf("【Motor_R】count:%i,angle_vel_R:%3.2f,pwm_R:%3.2f\n", 
        count_R, angle_vel_R,outputs.RightMotorEffortCmd);
        printf("【Motor_R】count:%i,angle_vel_R:%3.2f,pwm_R:%3.2f\n", 
        count_L, angle_vel_L, outputs.LeftMotorEffortCmd);

        // Odom pub
        //odom_pub_.publish(odom_);
        rate.sleep();
    }
    reset();
}