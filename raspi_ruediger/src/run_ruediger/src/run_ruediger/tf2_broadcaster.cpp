#include "tf2_broadcaster.hpp"
Tf2_broadcaster::Tf2_broadcaster(void){
  robot_init_orientation.setRPY(0, 0, 0);
}

Tf2_broadcaster::~Tf2_broadcaster(void){}

void Tf2_broadcaster::set_rotation(double roll, double pitch, double yaw){
  double init_roll, init_pitch, init_yaw;
  tf2::Matrix3x3(robot_init_orientation).getRPY(init_roll, init_pitch, init_yaw);
  
  tf2::Quaternion q;
  q.setRPY(init_roll + roll, init_pitch + pitch, init_yaw + yaw);

  tf2::convert(q, tf_stamped.transform.rotation);
}

void Tf2_broadcaster::set_rotation(tf2::Quaternion q){
  q *= robot_init_orientation;
  q.normalize();
  tf_stamped.transform.rotation.x = q.x();
  tf_stamped.transform.rotation.y = q.y();
  tf_stamped.transform.rotation.z = q.z();
  tf_stamped.transform.rotation.w = q.w();
}

void Tf2_broadcaster::set_rotation(geometry_msgs::Quaternion q){
  tf2::Quaternion q_orig;
  tf2::convert(q , q_orig);

  q_orig *= robot_init_orientation;
  q_orig.normalize();

  tf2::convert(q_orig, q);

  tf_stamped.transform.rotation.x = q.x;
  tf_stamped.transform.rotation.y = q.y;
  tf_stamped.transform.rotation.z = q.z;
  tf_stamped.transform.rotation.w = q.w;
}

void Tf2_broadcaster::set_translation(double x, double y, double z){
  tf_stamped.transform.translation.x = x;
  tf_stamped.transform.translation.y = y;
  tf_stamped.transform.translation.z = z;
}

void Tf2_broadcaster::set_translation(geometry_msgs::Vector3 translation){
  tf_stamped.transform.translation = translation;
}

void Tf2_broadcaster::loop(void){
  tf_stamped.header.stamp = ros::Time::now();
  tf_broadcaster.sendTransform(tf_stamped);
}