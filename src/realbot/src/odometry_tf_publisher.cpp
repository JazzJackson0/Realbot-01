#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <wiringPi.h>
#include <cmath>

float x = 0.0;
float y = 0.0;
float th = 0.0;
int left_encoder_pin = 27; // GPIO 27
int right_encoder_pin = 1; // GPIO 1
int full_revolution = 0; // Number of ticks for 1 wheel revolution.
float trackwidth = 0;
float wheel_diameter = 0.0;
float wheel_circumference = wheel_diameter * M_PI;
int left_wheel_tick_count = 0;
int right_wheel_tick_count = 0;

float* Calculate_Odometry()
void Update_LeftTick();
void Update_RightTick();


int main(int argc, char** argv) {

    ros::init(argc, argv, "odometry_tf_publisher");
    ros::NodeHandle = nh;
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);

    wiringPiSetup();
    pinMode(left_encoder_pin, INPUT);		
    pinMode(right_encoder_pin, INPUT);

    // Raspberry Pi Interrupt for Encoder Data Pins
    wiringPiISR(left_encoder_pin, INT_EDGE_RISING, Update_LeftTick); // Check Left Encoder Pin
    wiringPiISR(right_encoder_pin, INT_EDGE_RISING, Update_RightTick); // Ckeck Right Encoder Pin

    ros::Time current_time = ros::Time::now();
    ros::Time prev_time = ros::Time::now();
    tf::TransformBroadcaster tf_broadcaster;
    ros::Rate loop_rate(20);

    geometry_msgs::TransformStamped odom_transform;
    odom_transform.header.frame_id = "odom";
    odom_transform.child_frame_id = "base_footprint";

    while(ros::ok()) {

        float* new_reading = Calculate_Odometry();

        // Poulate Odometry Transform Message
        odom_transform.header.stamp = current_time;
        odom_transform.transform.translation.x = new_reading[0];
        odom_transform.transform.translation.y = new_reading[1];
        odom_transform.transform.translation.z = 0;
        odom_transform.transform.rotation = tf::createQuaternionMsgFromYaw(new_reading[2]);

        // Populate Odometry Message
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";
        odom.pose.pose.position.x = new_reading[0];
        odom.pose.pose.position.y = new_reading[1];
        odom.pose.pose.position.0.0;
        odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, new_reading[2]);
        odom.twist.twist.linear.x = 0.0;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.linear.z = 0.0;
        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;
        odom.twist.twist.angular.z = 0.0;
        prev_time = current_time;

        // Publish Data
        tf_broadcaster.sendTransform(odom_transform);
        odom_pub = publish(odom);

        loop_rate.sleep();
    }

    return 0;
}



/**
 * @brief Calculate Vehicle Odometry from Wheel Encoder Data
 * 
 * @return ** float* Updated robot coordinates
 */
float* Calculate_Odometry() {

    float new_coordinates[3];

    // Calculate Wheel Distances
    float left_wheel_distance = (left_wheel_tick_count / full_revolution) * wheel_circumference;
    float right_wheel_distance = (right_wheel_tick_count / full_revolution) * wheel_circumference;

    // Reset tick counts
    left_wheel_tick_count = 0;
    right_wheel_tick_count = 0;

    // Calculate Odometry
    float r_center = trackwidth / 2;

    float phi = (left_wheel_distance - right_wheel_distance) / trackwidth;
    float Px = x - r_center * cos(th);
    float Py = y - r_center * sin(th);

    new_coordinates[0] = Px + (r_center * cos(phi + th));
    new_coordinates[1] = Py + (r_center * sin(phi + th));
    new_coordinates[2] = phi + th;

    return new_coordinates;
}



/**
 * @brief Update the Left Wheel Encoder Tick Count
 * 
 * @return ** void 
 */
void Update_LeftTick() {

    left_wheel_tick_count++;
}



/**
 * @brief Update the Right Wheel Encoder Tick Count
 * 
 * @return ** void 
 */
void Update_RightTick() {

    right_wheel_tick_count++;
}