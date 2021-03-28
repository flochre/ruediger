#ifndef CAMERA_HPP
#define CAMERA_HPP

// These are the ROS headers for getting ROS Client API's.
#include <ros.h>
#include <pixy_msgs/PixyBlock.h>
#include <pixy_msgs/PixyData.h>

#include <sensor_msgs/Range.h>

#include "Pixy.h"

#define TIMER_CAMERA 100


class Camera {
private:
    unsigned long timer;

    // Creating handlers of Node, IMU message, quaternion and ROS publisher.
    ros::NodeHandle *nh_;
    ros::Publisher *cam_pub;

    // Limiting size to 30 to not overload the RAM of the arduino
    pixy_msgs::PixyBlock cam_block[PIXY_INITIAL_ARRAYSIZE];
    pixy_msgs::PixyData cam_msgs;

    uint16_t blocks;
    Pixy pixy;
public:
    Camera(void);
    ~Camera();

    unsigned long read_timer(void);
    void setup(ros::NodeHandle *nh, char topic_name[] = "cam_data");
    void loop(void);
};

#endif // CAMERA_HPP