#ifndef IMU_TF2_BROADCASTER_HPP
#define IMU_TF2_BROADCASTER_HPP

#include "tf2_broadcaster.hpp"

#include <std_msgs/Float32.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>

class Imu_tf2_broadcaster : public Tf2_broadcaster {
  private:
    // ros::NodeHandle *nh_;
    ros::Subscriber sub_imu_data;

    // tf2_ros::TransformBroadcaster imu_broadcaster;
    // geometry_msgs::TransformStamped transformStamped;

    // tf2::Quaternion robot_init_orientation;

  public:
    Imu_tf2_broadcaster(void);
    // Imu_tf2_broadcaster(std::string topic, std::string frame_id, std::string child_frame_id);
    ~Imu_tf2_broadcaster(void);

    // void tf2_callback(const std_msgs::Float32::ConstPtr& imu_msg);
    void tf2_callback(const geometry_msgs::Vector3::ConstPtr& imu_msg);
    // void tf2_callback(const sensor_msgs::Imu::ConstPtr& imu_msg);

    void setup(ros::NodeHandle *nh, std::string topic, std::string frame_id, std::string child_frame_id);
    void set_init_rotation(double roll, double pitch, double yaw);
    // void set_rotation(double roll, double pitch, double yaw);
    // void set_rotation(tf2::Quaternion q);
    // void set_rotation(geometry_msgs::Quaternion q);
    // void set_translation(double x, double y, double z);
    // void set_translation(geometry_msgs::Vector3 translation);

    // void loop(void);

};

#endif // IMU_TF2_BROADCASTER_HPP