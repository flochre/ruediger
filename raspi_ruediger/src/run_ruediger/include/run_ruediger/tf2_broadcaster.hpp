#ifndef TF2_BROADCASTER_HPP
#define TF2_BROADCASTER_HPP
#include <ros/ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>

#include <geometry_msgs/Vector3.h>

// void tf2_callback(const sensor_msgs::Imu::ConstPtr& imu_msg);

class Tf2_broadcaster {
  private:
    ros::NodeHandle *nh_;
    ros::Subscriber sub_imu_data;

    tf2_ros::TransformBroadcaster tfb;
    geometry_msgs::TransformStamped transformStamped;

    tf2::Quaternion robot_init_orientation;

  public:
    Tf2_broadcaster(void);
    Tf2_broadcaster(std::string topic, std::string frame_id, std::string child_frame_id);
    ~Tf2_broadcaster(void);

    void tf2_callback(const sensor_msgs::Imu::ConstPtr& imu_msg);

    void setup(ros::NodeHandle *nh, std::string topic, std::string frame_id, std::string child_frame_id);
    void set_init_rotation(double roll, double pitch, double yaw);
    void set_rotation(double roll, double pitch, double yaw);
    void set_rotation(tf2::Quaternion q);
    void set_rotation(geometry_msgs::Quaternion q);
    void set_translation(double x, double y, double z);
    void set_translation(geometry_msgs::Vector3 translation);

    void loop(void);

};

#endif // TF2_BROADCASTER_HPP