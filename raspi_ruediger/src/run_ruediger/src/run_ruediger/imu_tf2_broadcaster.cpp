#include "imu_tf2_broadcaster.hpp"

// void Imu_tf2_broadcaster::tf2_callback(const std_msgs::Float32::ConstPtr& imu_msg){
//   // ROS_ERROR("I heard: [%s]", imu_msg->header.frame_id.c_str());
//   // printf("sending aussi\n");
//   set_rotation(0.0, 0.0, imu_msg->data);
// }

void Imu_tf2_broadcaster::tf2_callback(const geometry_msgs::Vector3::ConstPtr& imu_msg){
  set_rotation(imu_msg->y, -imu_msg->x, imu_msg->z);
}

// void Imu_tf2_broadcaster::tf2_callback(const sensor_msgs::Imu::ConstPtr& imu_msg){
//   // ROS_ERROR("I heard: [%s]", imu_msg->header.frame_id.c_str());
//   // printf("sending aussi\n");
//   set_rotation(imu_msg->orientation);
// }

Imu_tf2_broadcaster::Imu_tf2_broadcaster(void){
  robot_init_orientation.setRPY(0, 0, 0);
}

Imu_tf2_broadcaster::Imu_tf2_broadcaster(std::string topic, std::string frame_id, std::string child_frame_id){
  robot_init_orientation.setRPY(0, 0, 0);
}

Imu_tf2_broadcaster::~Imu_tf2_broadcaster(void){}

void Imu_tf2_broadcaster::setup(ros::NodeHandle *nh, std::string topic, std::string frame_id, std::string child_frame_id){
  nh_ = nh;
  
  sub_imu_data = nh_->subscribe(topic.c_str(), 1000, &Imu_tf2_broadcaster::tf2_callback, this);

  transformStamped.header.frame_id = frame_id;
  transformStamped.child_frame_id = child_frame_id;
}

void Imu_tf2_broadcaster::set_init_rotation(double roll, double pitch, double yaw){
  robot_init_orientation.setRPY(roll, pitch, yaw);
}

void Imu_tf2_broadcaster::set_rotation(double roll, double pitch, double yaw){
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);

  q *= robot_init_orientation; 
  q.normalize();
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();
}

void Imu_tf2_broadcaster::set_rotation(tf2::Quaternion q){
  q *= robot_init_orientation;
  q.normalize();
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();
}

void Imu_tf2_broadcaster::set_rotation(geometry_msgs::Quaternion q){
  tf2::Quaternion q_orig;
  tf2::convert(q , q_orig);

  q_orig *= robot_init_orientation;
  q_orig.normalize();

  tf2::convert(q_orig, q);

  transformStamped.transform.rotation.x = q.x;
  transformStamped.transform.rotation.y = q.y;
  transformStamped.transform.rotation.z = q.z;
  transformStamped.transform.rotation.w = q.w;
}

void Imu_tf2_broadcaster::set_translation(double x, double y, double z){
  transformStamped.transform.translation.x = x;
  transformStamped.transform.translation.y = y;
  transformStamped.transform.translation.z = z;
}

void Imu_tf2_broadcaster::set_translation(geometry_msgs::Vector3 translation){
  transformStamped.transform.translation = translation;
}

void Imu_tf2_broadcaster::loop(void){
  transformStamped.header.stamp = ros::Time::now();
  imu_broadcaster.sendTransform(transformStamped);
}