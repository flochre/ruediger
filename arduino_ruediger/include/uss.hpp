#ifndef USS_HPP
#define USS_HPP

#include "MeUltrasonicSensor.h"

#include <ros.h>
#include <sensor_msgs/Range.h>

class Uss {

    ros::NodeHandle *nh_;
    ros::Publisher *uss_pub;
    sensor_msgs::Range uss_msg;
    MeUltrasonicSensor *me_uss;

  public:
    Uss(void);
    Uss(uint8_t port);
    void setup(ros::NodeHandle *nh, char topic_name[] = "uss_data");
    // void setup(ros::NodeHandle *nh, String topic_name);
    void loop(void);
};

#endif // USS_HPP