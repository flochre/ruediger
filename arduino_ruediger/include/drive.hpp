#ifndef DRIVE_HPP
#define DRIVE_HPP

// These are the ROS headers for getting ROS Client API's.
#include <ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>

class Drive {

  // Creating handlers of Node, IMU message, quaternion and ROS publisher.
  ros::NodeHandle *nh_;
  ros::Subscriber<geometry_msgs::Twist> *cmd_vel;

  public:
    Drive(void);

    void setup(ros::NodeHandle *nh, char sub_cmd_vel[], ros::Subscriber<geometry_msgs::Twist>::CallbackT cb_cmd_vel);
    void loop(void);
};

#endif // DRIVE_HPP