#include "base_robot_control_drv8833.hpp"
#include "drv8833.hpp"

int BaseRobotControl_DRV8833::pi;
int BaseRobotControl_DRV8833::EN_R_A;
int BaseRobotControl_DRV8833::EN_R_B;
int BaseRobotControl_DRV8833::EN_L_A;
int BaseRobotControl_DRV8833::EN_L_B;
int BaseRobotControl_DRV8833::count_R;
int BaseRobotControl_DRV8833::count_L;

BaseRobotControl_DRV8833::BaseRobotControl_DRV8833(ros::NodeHandle nh, ros::NodeHandle pnh){
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

    //ROS
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("/odom",5);
    vel_sub_ = nh.subscribe("/cmd_vel", 10, &BaseRobotControl_DRV8833::cmd_vel_callback, this);
    imu_sub_ = nh.subscribe("/imu/data", 10, &BaseRobotControl_DRV8833::imu_callback, this);
    process_timer_ = nh.createWallTimer(ros::WallDuration(MAIN_PROCESS_PERIOD),&BaseRobotControl_DRV8833::timer_callback,this);

    // For PID debug
    vel_pub_R_ = nh.advertise<geometry_msgs::Twist>("/motor_vel_R",5);
    vel_pub_L_ = nh.advertise<geometry_msgs::Twist>("/motor_vel_L",5);
    PID_sub_R_ = nh.subscribe("/PID_R",10,&BaseRobotControl_DRV8833::PID_R_callback,this);
    PID_sub_L_ = nh.subscribe("/PID_L",10,&BaseRobotControl_DRV8833::PID_L_callback,this);

    driver = new DRV8833();
    driver->setPinMode(pi, MOTOR_DRIVER_LI1, MOTOR_DRIVER_LI2, MOTOR_DRIVER_RI1, MOTOR_DRIVER_RI2);

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
    
    //Initialize PID
    diff_R = 0.0;
    diff_pre_R = 0.0;
    integral_R = 0.0;
    differential_R = 0.0;

    diff_L = 0.0;
    diff_pre_L = 0.0;
    integral_L = 0.0;
    differential_L = 0.0;

    //Timer callback debug
    pre_time = ros::WallTime::now();
    ROS_INFO("Initialized");
}

void BaseRobotControl_DRV8833::encoder_count_R_A(){
    if (digitalRead(EN_R_A) == digitalRead(EN_R_B)){
        count_R --;
    }else{
        count_R ++;
    }
}

void BaseRobotControl_DRV8833::encoder_count_R_B(){
    if (digitalRead(EN_R_A) != digitalRead(EN_R_B)){
        count_R --;
    }else{
        count_R ++;
    }
}

void BaseRobotControl_DRV8833::encoder_count_L_A(){
    if (digitalRead(EN_L_A) == digitalRead(EN_L_B)){
        count_L ++;
    }else{
        count_L --;
    }
}

void BaseRobotControl_DRV8833::encoder_count_L_B(){
    if (digitalRead(EN_L_A) != digitalRead(EN_L_B)){
        count_L ++;
    }else{
        count_L --;
    }
}

void BaseRobotControl_DRV8833::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr &vel){
    std::lock_guard<std::mutex> lock(m);
    target_vel_R = vel->linear.x + vel->angular.z * WHEEL_DIST;
    target_vel_L = vel->linear.x - vel->angular.z * WHEEL_DIST;
}

void BaseRobotControl_DRV8833::imu_callback(const sensor_msgs::Imu::ConstPtr &imu){
    std::lock_guard<std::mutex> lock(m);
    //robot pose contorol
}

void BaseRobotControl_DRV8833::PID_R_callback(const std_msgs::Float32MultiArray &msg){
    int num = msg.data.size();
    if (num !=3){
        ROS_INFO("Wrong array size /PID_R");
        return;
    } 
    KP_R = msg.data[0];
    KI_R = msg.data[1];
    KD_R = msg.data[2];
}

void BaseRobotControl_DRV8833::PID_L_callback(const std_msgs::Float32MultiArray &msg){
    int num = msg.data.size();
    if (num !=3){
        ROS_INFO("Wrong array size /PID_L");
        return;
    } 
    KP_L = msg.data[0];
    KI_L = msg.data[1];
    KD_L = msg.data[2];
}

float BaseRobotControl_DRV8833::calc_angle_output(int _count){
    float count_temp = 360.0 / (count_turn_out) * (_count % (count_turn_out));
    if (count_temp >= 0)
        return count_temp;
    else
        return count_temp + 360.0;
}

void BaseRobotControl_DRV8833::calc_odom(){
    float v_x, v_y, v_th; // m/s, m/s, rad/s

    // use differential two-wheeled robot equation
    v_x = (vel_R + vel_L)/2 * std::cos(odom_th);
    v_y = (vel_R + vel_L)/2 * std::sin(odom_th);
    v_th = (vel_R - vel_L)/ (2*WHEEL_DIST);

    odom_x = odom_x + v_x * MAIN_PROCESS_PERIOD * std::cos(odom_th + v_th * MAIN_PROCESS_PERIOD / 2);
    odom_y = odom_y + v_y * MAIN_PROCESS_PERIOD * std::sin(odom_th + v_th * MAIN_PROCESS_PERIOD / 2);
    odom_th = odom_th + v_th * MAIN_PROCESS_PERIOD;

    geometry_msgs::Quaternion odom_q = tf::createQuaternionMsgFromYaw(odom_th);
    odom_.header.frame_id = "odom";
    odom_.pose.pose.position.x = odom_x;
    odom_.pose.pose.position.y = odom_y;
    odom_.pose.pose.position.z = 0;
    odom_.pose.pose.orientation = odom_q;
}

void BaseRobotControl_DRV8833::timer_callback(const ros::WallTimerEvent &e){
    std::lock_guard<std::mutex> lock(m);
    //Timer debug
    //time = ros::WallTime::now();
    //ros::WallDuration time_diff = time - pre_time;
    //ROS_INFO("Timer Callback time : %u.%09u", time_diff.sec, time_diff.nsec);
    //pre_time = time;
    //Motor R
    angle_out_R = calc_angle_output(count_R);
    angle_vel_R = (360.0 * (count_R - count_R_pre) / count_turn_out / MAIN_PROCESS_PERIOD);
    count_R_pre = count_R;
    //Motor L
    angle_out_L = calc_angle_output(count_L);
    angle_vel_L = (360.0 * (count_L - count_L_pre) / count_turn_out / MAIN_PROCESS_PERIOD);
    count_L_pre = count_L;
    //low path filter
    angle_vel_R = a_vel * angle_vel_R + (1 - a_vel) * angle_vel_R_pre;
    angle_vel_L = a_vel * angle_vel_L + (1 - a_vel) * angle_vel_L_pre;
    angle_vel_R_pre = angle_vel_R;
    angle_vel_L_pre = angle_vel_L;
    //Calculate vel
    vel_R = WHEEL_DIA / 2.0 * (angle_vel_R / 360.0 *PI);
    vel_L = WHEEL_DIA / 2.0 * (angle_vel_L / 360.0 *PI);
    vel_R = a_vel * vel_R + (1 - a_vel) * vel_R_pre;
    vel_L = a_vel * vel_L + (1 - a_vel) * vel_L_pre;
    vel_R_pre = vel_R;
    vel_L_pre = vel_L;
    //For PID debug
    geometry_msgs::Twist motor_vel_R, motor_vel_L;
    motor_vel_R.linear.x = vel_R;
    motor_vel_L.linear.x = vel_L;
    vel_pub_R_.publish(motor_vel_R);
    vel_pub_L_.publish(motor_vel_L);

    calc_odom();
    motor_control();
}

void BaseRobotControl_DRV8833::motor_stop(){
    driver->drive(driver->A, 0);
    driver->drive(driver->B, 0);
    printf("Force motor stop\n");
    exit(1);
}

void BaseRobotControl_DRV8833::motor_control(){
    //bool dir_R, dir_L;

    // PID
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

    driver->drive(driver->A, pwm_L);
    driver->drive(driver->B, pwm_R);
}

void BaseRobotControl_DRV8833::main_loop(){
    //Motor start
    //gpio_write(pi, MOTOR_DRIVER_EN, PI_LOW);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate rate(5);
    ROS_INFO("Start Loop");
    while (ros::ok())
    {
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
