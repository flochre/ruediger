#ifndef USS_HPP
#define USS_HPP

#include "MeUltrasonicSensor.h"

#include <ros.h>
#include <sensor_msgs/Range.h>

#define TIMER_USS 1000

class Uss {

    ros::NodeHandle *nh_;
    ros::Publisher *uss_pub;
    sensor_msgs::Range uss_msg;
    MeUltrasonicSensor *me_uss;
    unsigned long timer;

  public:
    Uss(void);
    Uss(uint8_t port);
    unsigned long read_timer(void);
    void setup(ros::NodeHandle *nh, char topic_name[] = "uss_data");
    void loop(void);
};

#endif // USS_HPP