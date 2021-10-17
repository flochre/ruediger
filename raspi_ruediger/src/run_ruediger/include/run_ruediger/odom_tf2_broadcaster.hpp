#ifndef ODOM_TF2_BROADCASTER_HPP
#define ODOM_TF2_BROADCASTER_HPP

#include "tf2_broadcaster.hpp"

// #include <ros/ros.h>

// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>
// #include <tf2_ros/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>


class Odom_tf2_broadcaster: public Tf2_broadcaster {
  private:
    ros::Publisher odom_pub;

    ros::Subscriber sub_odom_data_left;
    ros::Subscriber sub_odom_data_right;

    int current_ticks_left, last_ticks_left;
    int current_ticks_right, last_ticks_right;

    double base_width_m;

    double x, y, th;
    double vx, vy, vth;
    double d_left, d_right;
    double v_left, v_right;

    ros::Time current_time, last_time;
    ros::Time current_time_left, last_time_left;
    ros::Time current_time_right, last_time_right;

    nav_msgs::Odometry odom;

  public:
    Odom_tf2_broadcaster(void);
    ~Odom_tf2_broadcaster(void);

    void encoder_left_callback(const std_msgs::Int32::ConstPtr& encoder_msg);
    void encoder_right_callback(const std_msgs::Int32::ConstPtr& encoder_msg);

    void setup(ros::NodeHandle *nh, std::string topic_left, std::string topic_right, std::string frame_id, std::string child_frame_id);
    void set_init_rotation(double roll, double pitch, double yaw);
    void set_init_translation(double x, double y);

    void loop(void);

};

#endif // ODOM_TF2_BROADCASTER_HPP