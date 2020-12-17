#ifndef IMU_HPP
#define IMU_HPP

#include "Wire.h"
#include "I2Cdev.h"
#include "helper_3dmath.h"

// #include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MeGyro.h"
// #include "Simple_MPU6050.h"

#include <ros.h>

//These are the headers to access IMU-ROS message header, quaternion message header and TF broadcaster
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>

#define TIMER_IMU 50
// #define MPU

class Imu {
//    public: 

    unsigned long timer;
    // These are the object handlers of TF message and broadcaster 
    // geometry_msgs::TransformStamped t;
    // tf::TransformBroadcaster *broadcaster;

    // Creating handlers of Node, IMU message, quaternion and ROS publisher.
    ros::NodeHandle *nh_;
    sensor_msgs::Imu imu_msgs;
    geometry_msgs::Quaternion orient;
    ros::Publisher *imu_pub;

    // MPU control/status vars
    int sensorPin;
    bool mpuInterrupt;
    bool dmpReady;          // set true if DMP init was successful
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer

    String frame_id = "/base_link";

    // orientation/motion vars
    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorInt16 aa;         // [x, y, z]            accel sensor measurements
    // VectorFloat aa;         // [x, y, z]            accel sensor measurements
    VectorFloat aaReal;     // [x, y, z]            gravity-free accel sensor measurements
    VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
    VectorFloat gravity;    // [x, y, z]            gravity vector
    float euler[3];         // [psi, theta, phi]    Euler angle container
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
    float ypr_rate[3];      // [yaw, pitch, roll]   yaw/pitch/roll (rates/speed) container vector

    MPU6050 mpu;
    MeGyro *gyro;


  public:
    Imu(void);
    unsigned long read_timer(void);
    void setup(ros::NodeHandle *nh, char topic_name[] = "imu_data");
    void loop(void);

    //The frame_id helps to visulize the Transform data of IMU w.r.t this link
    // char frameid[] = "/base_link";
    // char child[] = "/map";
};

#endif // IMU_HPP