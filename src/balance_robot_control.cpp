#include "balance_robot_control.h"

BalanceRobotControl::BalanceRobotControl(ros::NodeHandle nh){

    //ROS
    node_handle_ = nh;
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("/odom",1000);
    vel_sub_ = nh.subscribe("/cmd_vel", 10, &BalanceRobotControl::cmd_vel_callback, this);
    imu_sub_ = nh.subscribe("/imu/data", 10, &BalanceRobotControl::imu_callback, this);
    walltimer_ = nh.createWallTimer(ros::WallDuration(PROCESS_PERIOD),&BalanceRobotControl::timer_callback,this);


    // pigpio
    //GPIO setup -> Encoder
    pi = pigpio_start(0,0);
    if ( pi < 0 ){
        ROS_ERROR("pigpio Initialize Error (forget $sudo pigpiod?)\n");
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
    target_vel_R = vel->linear.x + vel->angular.z * WHEEL_DIST;
    target_vel_L = vel->linear.x - vel->angular.z * WHEEL_DIST;
}

void BalanceRobotControl::imu_callback(const sensor_msgs::Imu::ConstPtr &imu){
    //robot pose contorol
}

float BalanceRobotControl::calc_angle_output(int _count){
    int count_temp = 360.0 / (count_turn_out) * (_count % (count_turn_out));
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
    //Motor R
    angle_out_R = calc_angle_output(count_R);
    angle_vel_R = 360 * (count_R - count_R_pre) / count_turn_out / PROCESS_PERIOD;
    count_R_pre = count_R;
    //Motor L
    angle_out_L = calc_angle_output(count_L);
    angle_vel_L = 360 * (count_L - count_L_pre) / count_turn_out / PROCESS_PERIOD;
    count_L_pre = count_L;
    //Calculate vel
    vel_R = WHEEL_DIA / 2 * (angle_vel_R / 360 *PI);
    vel_L = WHEEL_DIA / 2 * (angle_vel_L / 360 *PI);
    calc_odom();
}

void BalanceRobotControl::stop(){
    gpio_write(pi, MOTOR_DRIVER_EN, PI_HIGH);
    printf("Force motor stop\n");
    exit(1);
}

void BalanceRobotControl::motor_control(){
    bool dir_R, dir_L;

    // PID
    pwm_R = pwm_R +KP_R*(target_vel_R - vel_R);
    pwm_L = pwm_L +KP_L*(target_vel_L - vel_L);

    printf("target_vel_R:%3.1f, target_vel_L:%3.1f\n", target_vel_R, target_vel_L);
    printf("vel_R:%3.1f, vel_L:%3.1f\n", vel_R, vel_L);
    printf("pwm_R:%3.1f, pwm_L:%3.1f\n",pwm_R, pwm_L);

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
    ros::Rate rate(5);

    while (ros::ok())
    {
        printf("Motor_R:count %i,angle_out %3.1f,speed %3.1f \n", count_R, angle_out_R, angle_vel_R);
        printf("Motor_L:count %i,angle_out %3.1f,speed %3.1f \n\n", count_L, angle_out_L, angle_vel_L);

        motor_control();

        odom_pub_.publish(odom_);

        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "balance_robot_control");
    ros::NodeHandle nh;
    BalanceRobotControl control(nh);
    control.main_loop();
}