#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "MeEncoderOnBoard.h"

#include <ros.h>
#include <std_msgs/Int32.h>

class Motor {
    ros::NodeHandle *nh_;

    std_msgs::Int32 encoder_info;

    ros::Publisher *motor_pub;
    ros::Subscriber<std_msgs::Int32> *motor_sub;

    int32_t my_speed;

  public:
    MeEncoderOnBoard my_motor;
    void isr_process_encoder(void);
    void motor_msg(const std_msgs::Int32& msg);
    void configure_motor(int pulse, float ratio, float pos_p, float pos_i,float pos_d, float speed_p, float speed_i,float speed_d);
    void set_default_values(long pulse_pos, long position, float speed, int16_t pwm, int16_t motionMode);
    void set_speed(int32_t speed);

    Motor(void);
    Motor(uint8_t slot);
    void reset(uint8_t slot);
    void setup(ros::NodeHandle *nh, char publisher_name[]);
    void setup(ros::NodeHandle *nh, char publisher_name[], char subscriber_name[], ros::Subscriber<std_msgs::Int32>::CallbackT cb);
    void loop(void);
};

#endif // MOTOR_HPP