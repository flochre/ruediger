#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "MeEncoderOnBoard.h"

#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>

class Motor {
    ros::NodeHandle *nh_;
    ros::Subscriber<std_msgs::Int32> *motor_sub;

  public:
    MeEncoderOnBoard my_motor;
    void isr_process_encoder(void);
    void motor_msg(const std_msgs::Int32& msg);

  public:
    Motor(void);
    Motor(uint8_t slot);
    void setup(ros::NodeHandle *nh, char topic_name[], ros::Subscriber<std_msgs::Int32>::CallbackT cb);
    void loop(void);
};

#endif // MOTOR_HPP