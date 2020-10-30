#include "balance_robot_control.h"

BalanceRobotControl::BalanceRobotControl(ros::NodeHandle nh){

    //ROS
    node_handle_ = nh;
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("/odom",5);
    vel_sub_ = nh.subscribe("/cmd_vel", 10, &BalanceRobotControl::cmd_vel_callback, this);
    imu_sub_ = nh.subscribe("/imu/data", 10, &BalanceRobotControl::imu_callback, this);
    process_timer_ = nh.createWallTimer(ros::WallDuration(PROCESS_PERIOD),&BalanceRobotControl::timer_callback,this);

    // For PID debug
    vel_pub_R_ = nh.advertise<geometry_msgs::Twist>("/motor_vel_R",5);
    vel_pub_L_ = nh.advertise<geometry_msgs::Twist>("/motor_vel_L",5);
    PID_sub_R_ = nh.subscribe("/PID_R",10,&BalanceRobotControl::PID_R_callback,this);
    PID_sub_L_ = nh.subscribe("/PID_L",10,&BalanceRobotControl::PID_L_callback,this);

    // pigpio
    //GPIO setup -> Encoder
    pi = pigpio_start(0,0);
    if ( pi < 0 ){
        ROS_ERROR("pigpio Initialize Error (Forget $sudo pigpiod?)\n");
        exit(1);
    }

    /*
    set_mode(pi, EN_R_A, PI_INPUT);
    set_mode(pi, EN_R_B, PI_INPUT);
    callback(pi, EN_R_A, EITHER_EDGE ,&encoder_count_R_A);
    callback(pi, EN_R_B, EITHER_EDGE ,&encoder_count_R_B);

    set_mode(pi, EN_L_A, PI_INPUT);
    set_mode(pi, EN_L_B, PI_INPUT);
    callback(pi, EN_L_A, EITHER_EDGE ,&encoder_count_L_A);
    callback(pi, EN_L_B, EITHER_EDGE ,&encoder_count_L_B);
    */

    //GPIO setup -> Motor driver
    set_mode(pi, MOTOR_DRIVER_EN, PI_OUTPUT);
    gpio_write(pi, MOTOR_DRIVER_EN, PI_HIGH);
    set_mode(pi, MOTOR_DRIVER_FAULT, PI_INPUT);
    //callback(pi, MOTOR_DRIVER_FAULT, EITHER_EDGE, &stop_motor);

    set_mode(pi, MOTOR_DIR_R, PI_OUTPUT);
    gpio_write(pi, MOTOR_DIR_R, PI_LOW);
    set_PWM_range(pi, MOTOR_PWM_R, PWM_RANGE);
    set_PWM_frequency(pi, MOTOR_PWM_R, MOTOR_FREQ);

    set_mode(pi, MOTOR_DIR_L, PI_OUTPUT);
    gpio_write(pi, MOTOR_DIR_L, PI_LOW);
    set_PWM_range(pi, MOTOR_PWM_L, PWM_RANGE);
    set_PWM_frequency(pi, MOTOR_PWM_L, MOTOR_FREQ);


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

    /*
    //GPIO setup -> Motor driver
    pinMode(MOTOR_DRIVER_EN, OUTPUT); // 0-> motor start, 1-> motor stop
    digitalWrite(MOTOR_DRIVER_EN, 1);
    //wiringPiISR(MOTOR_DRIVER_FAULT, INT_EDGE_RISING, stop_motor); // 0-> motor over-temperture or over-current, 1-> normally 

    pinMode(MOTOR_DIR_R, OUTPUT);
    digitalWrite(MOTOR_DIR_R, 0);
    pinMode(MOTOR_PWM_R,PWM_OUTPUT);

    pinMode(MOTOR_DIR_L, OUTPUT);
    digitalWrite(MOTOR_DIR_L, 0);
    pinMode(MOTOR_PWM_L,PWM_OUTPUT);

    pwmSetMode(0);
    pwmSetRange(1024);
    */

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
    vel_R = 0.0; //[m/s]
    target_vel_R = 0.0; //[m/s]
    pwm_R = 0.0; // 0 ~ PWM_RANGE

    vel_L = 0.0; //[m/s]
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
}

void BalanceRobotControl::encoder_count_R_A(){
    if (digitalRead(EN_R_A) == digitalRead(EN_R_B)){
        count_R --;
    }else{
        count_R ++;
    }
}
void BalanceRobotControl::encoder_count_R_A(int, unsigned int, unsigned int, unsigned int){
    if (gpio_read(pi, EN_R_A) == gpio_read(pi, EN_R_B)){
        count_R --;
    }else{
        count_R ++;
    }
}

void BalanceRobotControl::encoder_count_R_B(){
    if (digitalRead(EN_R_A) != digitalRead(EN_R_B)){
        count_R --;
    }else{
        count_R ++;
    }
}
void BalanceRobotControl::encoder_count_R_B(int, unsigned int, unsigned int, unsigned int){
    if (gpio_read(pi, EN_R_A) != gpio_read(pi, EN_R_B)){
        count_R --;
    }else{
        count_R ++;
    }
}

void BalanceRobotControl::encoder_count_L_A(){
    if (digitalRead(EN_L_A) == digitalRead(EN_L_B)){
        count_L --;
    }else{
        count_L ++;
    }
}
void BalanceRobotControl::encoder_count_L_A(int, unsigned int, unsigned int, unsigned int){
    if (gpio_read(pi, EN_L_A) == gpio_read(pi, EN_L_B)){
        count_L --;
    }else{
        count_L ++;
    }
}

void BalanceRobotControl::encoder_count_L_B(){
    if (digitalRead(EN_L_A) != digitalRead(EN_L_B)){
        count_L --;
    }else{
        count_L ++;
    }
}
void BalanceRobotControl::encoder_count_L_B(int, unsigned int, unsigned int, unsigned int){
    if (gpio_read(pi, EN_L_A) != gpio_read(pi, EN_L_B)){
        count_L --;
    }else{
        count_L ++;
    }
}

void BalanceRobotControl::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr &vel){
    std::lock_guard<std::mutex> lock(m);
    target_vel_R = vel->linear.x + vel->angular.z * WHEEL_DIST;
    target_vel_L = vel->linear.x - vel->angular.z * WHEEL_DIST;
}

void BalanceRobotControl::imu_callback(const sensor_msgs::Imu::ConstPtr &imu){
    std::lock_guard<std::mutex> lock(m);
    //robot pose contorol
}

void BalanceRobotControl::PID_R_callback(const std_msgs::Float32MultiArray &msg){
    int num = msg.data.size();
    if (num !=3){
        ROS_INFO("Wrong array size /PID_R");
        return;
    } 
    KP_R = msg.data[0];
    KI_R = msg.data[1];
    KD_R = msg.data[2];
}

void BalanceRobotControl::PID_L_callback(const std_msgs::Float32MultiArray &msg){
    int num = msg.data.size();
    if (num !=3){
        ROS_INFO("Wrong array size /PID_L");
        return;
    } 
    KP_L = msg.data[0];
    KI_L = msg.data[1];
    KD_L = msg.data[2];
}

float BalanceRobotControl::calc_angle_output(int _count){
    float count_temp = 360.0 / (count_turn_out) * (_count % (count_turn_out));
    if (count_temp >= 0)
        return count_temp;
    else
        return count_temp + 360.0;
}

void BalanceRobotControl::calc_odom(){
    float v_x, v_y, v_th; // m/s, m/s, rad/s

    // use differential two-wheeled robot equation
    v_x = (vel_R + vel_L)/2 * std::cos(odom_th);
    v_y = (vel_R + vel_L)/2 * std::sin(odom_th);
    v_th = (vel_R - vel_L)/ (2*WHEEL_DIST);

    odom_x = odom_x + v_x * PROCESS_PERIOD * std::cos(odom_th + v_th * PROCESS_PERIOD / 2);
    odom_y = odom_y + v_y * PROCESS_PERIOD * std::sin(odom_th + v_th * PROCESS_PERIOD / 2);
    odom_th = odom_th + v_th * PROCESS_PERIOD;

    geometry_msgs::Quaternion odom_q = tf::createQuaternionMsgFromYaw(odom_th);
    odom_.header.frame_id = "odom";
    odom_.pose.pose.position.x = odom_x;
    odom_.pose.pose.position.y = odom_y;
    odom_.pose.pose.position.z = 0;
    odom_.pose.pose.orientation = odom_q;
}

void BalanceRobotControl::timer_callback(const ros::WallTimerEvent &e){
    std::lock_guard<std::mutex> lock(m);
    //Timer debug
    //time = ros::WallTime::now();
    //ros::WallDuration time_diff = time - pre_time;
    //ROS_INFO("Timer Callback time : %u.%09u", time_diff.sec, time_diff.nsec);
    //pre_time = time;
    //Motor R
    angle_out_R = calc_angle_output(count_R);
    angle_vel_R = -1.0*(360.0 * (count_R - count_R_pre) / count_turn_out / PROCESS_PERIOD);
    count_R_pre = count_R;
    //Motor L
    angle_out_L = calc_angle_output(count_L);
    angle_vel_L = 360.0 * (count_L - count_L_pre) / count_turn_out / PROCESS_PERIOD;
    count_L_pre = count_L;
    //Calculate vel
    vel_R = WHEEL_DIA / 2.0 * (angle_vel_R / 360.0 *PI);
    vel_L = WHEEL_DIA / 2.0 * (angle_vel_L / 360.0 *PI);
    //For PID debug
    geometry_msgs::Twist motor_vel_R, motor_vel_L;
    motor_vel_R.linear.x = vel_R;
    motor_vel_L.linear.x = vel_L;
    vel_pub_R_.publish(motor_vel_R);
    vel_pub_L_.publish(motor_vel_L);

    calc_odom();
    motor_control();
}

void BalanceRobotControl::motor_stop(){
    gpio_write(pi, MOTOR_DRIVER_EN, PI_HIGH);
    printf("Force motor stop\n");
    exit(1);
}

void BalanceRobotControl::motor_control(){
    bool dir_R, dir_L;

    // PID
    diff_R = target_vel_R - vel_R;
    integral_R += (diff_R + diff_pre_R)/2.0*PROCESS_PERIOD;
    differential_R = (diff_R - diff_pre_R)/PROCESS_PERIOD;

    pwm_R = KP_R*diff_R + KI_R*integral_R + KD_R*differential_R;
    pwm_R = std::min(std::max(-1*PWM_RANGE,pwm_R),PWM_RANGE);

    diff_L = target_vel_L - vel_L;
    integral_L += (diff_L + diff_pre_L)/2.0*PROCESS_PERIOD;
    differential_L = (diff_L - diff_pre_L)/PROCESS_PERIOD;

    pwm_L = KP_L*diff_L + KI_L*integral_L + KD_L*differential_L;
    pwm_L = std::min(std::max(-1*PWM_RANGE,pwm_L),PWM_RANGE);

    // Decide dir by pwm code
    dir_R = (pwm_R >=0) ? PI_HIGH : PI_LOW;
    dir_L = (pwm_L >=0) ? PI_HIGH : PI_LOW;

    //Write PWM
    gpio_write(pi, MOTOR_DIR_R, dir_R);
    set_PWM_dutycycle(pi, MOTOR_PWM_R, abs(pwm_R));

    gpio_write(pi, MOTOR_DIR_L, dir_L);
    set_PWM_dutycycle(pi, MOTOR_PWM_L, abs(pwm_L));

}

void BalanceRobotControl::main_loop(){
    //Motor start
    gpio_write(pi, MOTOR_DRIVER_EN, PI_LOW);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate rate(5);
    while (ros::ok()){
        printf("【Motor_R】count:%i,angle_out:%3.2f,angle_vel_R:%3.2f,target_vel_R:%3.2f,vel_R:%3.2f,pwm_R:%3.2f\n", 
        count_R, angle_out_R, angle_vel_R,target_vel_R,vel_R,pwm_R);
        printf("【Motor_L】count:%i,angle_out:%3.2f,angle_vel_L:%3.2f,target_vel_L:%3.2f,vel_L:%3.2f,pwm_L:%3.2f\n\n",
        count_L, angle_out_L, angle_vel_L,target_vel_L,vel_L,pwm_L);
        odom_pub_.publish(odom_);
        rate.sleep();
    }
    motor_stop();
    spinner.stop();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "balance_robot_control");
    ros::NodeHandle nh;
    BalanceRobotControl control(nh);
    control.main_loop();
    return 0;
}