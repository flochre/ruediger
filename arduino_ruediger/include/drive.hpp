#ifndef DRIVE_HPP
#define DRIVE_HPP

// These are the ROS headers for getting ROS Client API's.
#include <ros.h>
#include <std_msgs/Int32.h>

class Drive {

    // Creating handlers of Node, IMU message, quaternion and ROS publisher.
    ros::NodeHandle *nh_;
    ros::Subscriber<std_msgs::Int32> *drive_vx;
    ros::Subscriber<std_msgs::Int32> *drive_vy;
    ros::Subscriber<std_msgs::Int32> *drive_vteta;

  public:
    Drive(void);

    // void vx_msg(const std_msgs::Int32& msg);
    // void vy_msg(const std_msgs::Int32& msg);
    // void vteta_msg(const std_msgs::Int32& msg);

    void setup(ros::NodeHandle *nh, char sub_vx_name[], char sub_vy_name[], char sub_vteta_name[], ros::Subscriber<std_msgs::Int32>::CallbackT cb_vx, ros::Subscriber<std_msgs::Int32>::CallbackT cb_vy, ros::Subscriber<std_msgs::Int32>::CallbackT cb_vteta);
    void loop(void);
};

#endif // DRIVE_HPP