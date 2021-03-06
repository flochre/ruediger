#ifndef USS_HPP
#define USS_HPP

#include "MeUltrasonicSensor.h"

#include <ros.h>
#include <sensor_msgs/Range.h>

#define TIMER_USS 250
#define SEC_2_MICROSEC 1e6
#define MICROSEC_2_SEC 1e-6
#define SOUND_SPEED 343 // meter per seconds

class Uss {
private:
  unsigned long timer;
  
  ros::NodeHandle *nh_;
  ros::Publisher *uss_pub;

  sensor_msgs::Range uss_msg;

  MeUltrasonicSensor *me_uss;
  float distance;
  float distance_meters(float max_m = 4.0);

public:
  Uss(void);
  Uss(uint8_t port);
  ~Uss();

  unsigned long read_timer(void);
  void setup(ros::NodeHandle *nh, char topic_name[] = "uss_data");
  void loop(void);
};

#endif // USS_HPP