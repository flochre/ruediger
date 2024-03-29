#ifndef DRIVE_HPP
#define DRIVE_HPP

// These are the ROS headers for getting ROS Client API's.
#include <ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>

#include "MePort.h"
#include "motor.hpp"

#define TIMER_DRIVER 80

void motor_2_cmd(const std_msgs::Int32 &msg);
void motor_3_cmd(const std_msgs::Int32 &msg);

void cmd_vel(const geometry_msgs::Twist &my_speed);

class Drive {
  unsigned long timer;

  // Creating handlers of Node, IMU message, quaternion and ROS publisher.
  ros::NodeHandle *nh_;
  ros::Subscriber<geometry_msgs::Twist> *cmd_vel;
  ros::Subscriber<std_msgs::Int32> *cmd_mot1;
  ros::Subscriber<std_msgs::Int32> *cmd_mot4;

  void cmd_vel_cb(const geometry_msgs::Twist& my_speed);

  public:
    Drive(void);

    Motor motor_1;
    Motor motor_2;
    Motor motor_3;
    Motor motor_4;

    unsigned long read_timer(void);
    void set_speed(const geometry_msgs::Twist& my_speed);
    // Setup the robot to drive on motor 2 and 3
    void setup(ros::NodeHandle *nh, char sub_cmd_vel[], ros::Subscriber<geometry_msgs::Twist>::CallbackT cb_cmd_vel);
    // Setup the robot to drive on motor 2 and 3 and allow the motor 1 and 4 to be commanded over ROS by PWM
    void setup(ros::NodeHandle *nh, char sub_cmd_vel[], ros::Subscriber<geometry_msgs::Twist>::CallbackT cb_cmd_vel, ros::Subscriber<std_msgs::Int32>::CallbackT cb_cmd_mot1, ros::Subscriber<std_msgs::Int32>::CallbackT cb_cmd_mot4);
    void setup(ros::NodeHandle *nh, char sub_cmd_vel[], ros::Subscriber<geometry_msgs::Twist>::CallbackT cb_cmd_vel, ros::Subscriber<std_msgs::Int32>::CallbackT cb_cmd_mot1, ros::Subscriber<std_msgs::Int32>::CallbackT cb_cmd_mot2, ros::Subscriber<std_msgs::Int32>::CallbackT cb_cmd_mot3, ros::Subscriber<std_msgs::Int32>::CallbackT cb_cmd_mot4);
    void loop(void);
};

#endif // DRIVE_HPP