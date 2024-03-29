#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "MeEncoderOnBoard.h"

#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

class Motor {
    ros::NodeHandle *nh_;

    std_msgs::Int32 encoder_info;
    std_msgs::Float32 motor_speed_info;

    ros::Publisher *encoder_pub;
    ros::Publisher *motor_speed_pub;
    ros::Subscriber<std_msgs::Int32> *motor_sub;

    int32_t my_speed;
    uint8_t m_reversed;

  public:
    MeEncoderOnBoard my_motor;
    void isr_process_encoder(void);
    void motor_msg(const std_msgs::Int32& msg);
    void configure_motor(int pulse, float ratio, float pos_p, float pos_i,float pos_d, float speed_p, float speed_i,float speed_d);
    void set_default_values(long pulse_pos, long position, float speed, int16_t pwm, int16_t motionMode);
    void set_speed(int32_t speed);

    Motor(void);
    Motor(uint8_t slot);
    void reset(uint8_t slot, uint8_t reversed = 0);
    // Setup only publisher
    void setup(ros::NodeHandle *nh, char publisher_name[]);
    // Setup only subscriber
    void setup(ros::NodeHandle *nh, char subscriber_name[], ros::Subscriber<std_msgs::Int32>::CallbackT cb);
    // Setup both
    void setup(ros::NodeHandle *nh, char publisher_encoder_name[], char publisher_motorspd_name[], char subscriber_name[], ros::Subscriber<std_msgs::Int32>::CallbackT cb);
    void loop(void);
};

#endif // MOTOR_HPP