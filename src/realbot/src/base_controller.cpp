#include <ros/ros.h>
#include <iostream>
#include <wiringPi.h>

//
int left_encoder_pin = 27; // GPIO 27
int right_encoder_pin = 1; // GPIO 1

int front_left_motor_speed_pin = 12; // GPIO 12
int front_left_motor_dir_pin1 = 5; // GPIO 5
int front_left_motor_dir_pin2 = 6; // GPIO 6

int front_right_motor_speed_pin = 13; // GPIO 13
int front_right_motor_dir_pin1 = 26; // GPIO 26
int front_right_motor_dir_pin2 = 16; // GPIO 16

int back_left_motor_speed_pin = 18; // GPIO 18
int back_left_motor_dir_pin1 = 17; // GPIO 17
int back_left_motor_dir_pin2 = 27; // GPIO 27

int back_right_motor_speed_pin = 24; // GPIO 24
int back_right_motor_dir_pin1 = 15; // GPIO 15
int back_right_motor_dir_pin2 = 16; // GPIO 16

float track_width = 0;
ros::Time = prev_time;

// 
ros::Time prev_left_trigger_time = 0;
ros::Time prev_right_trigger_time = 0;
float left_time_interval;
float right_time_interval;

// Calculating Wheel Travel Distance (For Wheel Speed Unit Converisons)
int full_revolution = 0; // Number of ticks for 1 wheel revolution.
float wheel_diameter = 0.0;
float wheel_circumference = wheel_diameter * M_PI;


// Declarations
void cmd_velCallback(const geometry_msgs::Twist &twist_aux);
void Update_LeftEncoderVelocity();
void Update_RightEncoderVelocity();
void Update_LeftPWM(std_msgs::Float64 pwm_val);
void Update_RightPWM(std_msgs::Float64 pwm_val);


int main(int argc char** argv) {

    ros::init(argc, argv, "base_controller");
    ros::NodeHandle nh;

    wiringPiSetup();

    pinMode(left_encoder_pin, INPUT);	// Might be redundant, since this is done by the odometry node	
    pinMode(right_encoder_pin, INPUT);  // Might be redundant, since this is done by the odometry node	

    pinMode(front_left_motor_speed_pin, OUTPUT);
    pinMode(front_left_motor_dir_pin1, OUTPUT);
    pinMode(front_left_motor_dir_pin2, OUTPUT);

    pinMode(front_right_motor_speed_pin, OUTPUT);
    pinMode(front_right_motor_dir_pin1, OUTPUT);
    pinMode(front_right_motor_dir_pin2, OUTPUT);

    pinMode(back_left_motor_speed_pin, OUTPUT);
    pinMode(back_left_motor_dir_pin1, OUTPUT);
    pinMode(back_left_motor_dir_pin2, OUTPUT);

    pinMode(back_right_motor_speed_pin, OUTPUT);
    pinMode(back_right_motor_dir_pin1, OUTPUT);
    pinMode(back_right_motor_dir_pin2, OUTPUT);

    ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 10, cmd_velCallback);
    ros::Subscriber left_pid_output = nh.subscribe("/left_wheel/controller/control_effort", 10, Update_LeftPWM);
    ros::Subscriber right_pid_output = nh.subscribe("/right_wheel/controller/control_effort", 10, Update_RightPWM);

    ros::Publisher left_pid_input = nh.advertise<std_msgs::Float64>("/left_wheel/controller/state", 10); 
    ros::Publisher right_pid_input = nh.advertise<std_msgs::Float64>("/right_wheel/controller/state", 10);
    ros::Publisher left_pid_setpoint = nh.advertise<std_msgs::Float64>("/left_wheel/controller/setpoint", 10);
    ros::Publisher right_pid_setpoint = nh.advertise<std_msgs::Float64>("/right_wheel/controller/setpoint", 10);

    ros::Rate loop_rate(10);

    // Raspberry Pi Interrupt for Encoder Data Pins
    wiringPiISR(left_encoder_pin, INT_EDGE_RISING, Update_LeftEncoderVelocity); // Check Left Encoder Pin
    wiringPiISR(right_encoder_pin, INT_EDGE_RISING, Update_RightEncoderVelocity); // Ckeck Right Encoder Pin

    while(ros::ok()) {

        ros::Time current_time = ros::Time::now();
        ros::spinOnce();

        dt = (current_time - prev_time).toSec();
        prev_time = current_time;

        loop_rate.sleep();
    }

    return 0;
}


/**
 * @brief Calculate the velocity SetPoint Values to be passed to the PIDs for each wheel (PID SetPoints).
 * 
 * @param twist_aux 
 * @return ** void 
 */
void cmd_velCallback(const geometry_msgs::Twist &twist_aux) {

    geometry_msgs::Twist twist = twist_aux;
    float forward_vel = twist_aux.linear.x;
    float rotation_vel = twist_aux.angular.z;
    float right_wheel_vel = 0.0;
    float left_wheel_vel = 0.0;

    if (forward_vel == 0) {

        // Initiate Turning Velocity
        right_wheel_vel = rotation_vel * track_width / 2.0;
        left_wheel_vel = -1 * right_wheel_vel;
    }

    else if (rotation_vel == 0) {

        // Initiate Forward / Backward Velocity
        left_wheel_vel = right_wheel_vel = forward_vel;
    }

    else {

        // Initiate Arc Movement
        left_wheel_vel = forward_vel - rotation_vel * (track_width / 2.0);
        right_wheel_vel = forward_vel + rotation_vel * (track_width / 2.0);
    }

    // Publish the PID SetPoints
    std_msgs::Float64 left_setpoint;
    std_msgs::Float64 right_setpoint;
    left_setpoint = left_wheel_vel;
    right_setpoint = right_wheel_vel;

    left_pid_setpoint = publish(left_setpoint);
    right_pid_setpoint = publish(right_setpoint);
}



/**
 * @brief Calculate the velocity (m/sec) of the left wheel (PID Process Value).
 *        Publish the velocity data to the "state" topic.
 * 
 * @return ** void 
 */
void Update_LeftEncoderVelocity() {

   ros::Time current_trigger_time = ros::Time::now();
   left_time_interval = (prev_left_trigger_time - current_trigger_time).toSec();
   prev_left_trigger_time = current_trigger_time;
   float left_m_per_sec = wheel_circumference /  (full_revolution / (1 / left_time_interval)); // The motor velocity. Converted (ticks/sec) --> (m/sec)

    // Publish the Process Value
    std_msgs::Float64 left_process_val;
    left_process_val = left_m_per_sec;
    left_pid_input = publish(left_process_val);
}



/**
 * @brief Calculate the velocity (m/sec) of the right wheel (PID Process Value)
 *        Publish the velocity data to the "state" topic.
 * 
 * @return ** void 
 */
void Update_RightEncoderVelocity() {

   ros::Time current_trigger_time = ros::Time::now();
   right_time_interval = (prev_right_trigger_time - current_trigger_time).toSec();
   prev_right_trigger_time = current_trigger_time;
   float right_m_per_sec = wheel_circumference /  (full_revolution / (1 / right_time_interval)); // The motor velocity. Converted (ticks/sec) --> (m/sec)

   // Publish the Process Value
    std_msgs::Float64 right_process_val;
    right_process_val = right_m_per_sec;
    right_pid_input = publish(right_process_val);
}



/**
 * @brief Update the PWM output values for the front and rear left wheels
 * 
 * @param pwm_val 
 * @return ** void 
 */
void Update_LeftPWM(std_msgs::Float64 pwm_val) {

    pwmWrite(front_left_motor_speed_pin, pwm_val);
    pwmWrite(back_left_motor_speed_pin, pwm_val);
}



/**
 * @brief Update the PWM output values for the front and rear right wheels
 * 
 * @param pwm_val 
 * @return ** void 
 */
void Update_RightPWM(std_msgs::Float64 pwm_val) {

    pwmWrite(front_right_motor_speed_pin, pwm_val);
    pwmWrite(back_right_motor_speed_pin, pwm_val);
}
