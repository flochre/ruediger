#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "MeEncoderOnBoard.h"

#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>

class Motor {

    ros::NodeHandle *nh_;
    std_msgs::Int16 *mot_msg;
    // ros::Subscriber<std_msgs::Int16> *motor_sub;
    // ros::Subscriber<std_msgs::Int16, Motor> sub;

    // ros::Subscriber<std_msgs::Int32> sub("rand_no", &messageCb);
    ros::Subscriber<std_msgs::Int32> *sub;


  public:
    MeEncoderOnBoard *me_motor;

    // static void isr_process_encoder(void);
    void motor_msg(const std_msgs::Int16& msg);

    void inc(void);
    void dec(void);
    void getPortB(void);

  public:
    Motor(void);
    Motor(uint8_t slot);
    // void setup(ros::NodeHandle *nh, String topic_name);
    void setup(ros::NodeHandle *nh, String topic_name);
    void loop(void);
};

#endif // MOTOR_HPP