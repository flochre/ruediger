#ifndef TF2_BROADCASTER_HPP
#define TF2_BROADCASTER_HPP

#include <ros/ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
// #include "LinearMath/btMatrix3x3.h"
// #include <Matrix3x3.h>

class Tf2_broadcaster {
  protected:
    ros::NodeHandle *nh_;
    ros::Subscriber subscriber_data;

    tf2_ros::TransformBroadcaster tf_broadcaster;
    geometry_msgs::TransformStamped tf_stamped;

    tf2::Quaternion robot_init_orientation;

  public:
    Tf2_broadcaster(void);
    ~Tf2_broadcaster(void);

    void set_rotation(double roll, double pitch, double yaw);
    void set_rotation(tf2::Quaternion q);
    void set_rotation(geometry_msgs::Quaternion q);

    void set_translation(double x, double y, double z);
    void set_translation(geometry_msgs::Vector3 translation);

    void loop(void);

};

#endif // TF2_BROADCASTER_HPP