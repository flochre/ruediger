#ifndef IMU_HPP
#define IMU_HPP

#include "Wire.h"
#include "I2Cdev.h"
#include "helper_3dmath.h"
#include "MeGyro.h"

#include <ros.h>

//These are the headers to access IMU-ROS message header, quaternion message header and TF broadcaster
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>

#define TIMER_IMU 50

class Imu {
//    public: 

    unsigned long timer;
    // These are the object handlers of TF message and broadcaster 
    geometry_msgs::TransformStamped t;
    tf::TransformBroadcaster *broadcaster;

    // Creating handlers of Node, IMU message, quaternion and ROS publisher.
    ros::NodeHandle *nh_;
    sensor_msgs::Imu imu_msgs;
    geometry_msgs::Quaternion orient;
    ros::Publisher *imu_pub;

    //The frame_id helps to visulize the Transform data of IMU w.r.t this link
    char frameid[] = "/base_link";
    char child[] = "/map";

    // MPU control/status vars
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint8_t fifoBuffer[64]; // FIFO storage buffer

    // orientation/motion vars
    Quaternion q;           // [w, x, y, z]         quaternion container

    // packet structure for InvenSense teapot demo
    // uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

    MeGyro *gyro;

  public:
    Imu(void);
    unsigned long read_timer(void);
    void setup(ros::NodeHandle *nh, char topic_name[] = "imu_data");
    void loop(void);
};

#endif // IMU_HPP