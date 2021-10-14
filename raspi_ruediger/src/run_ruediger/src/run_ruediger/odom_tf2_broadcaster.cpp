#include "odom_tf2_broadcaster.hpp"

double ticks_to_meters(int wheel_ticks, int gear_ratio = 46, int ticks_per_rounds = 8 , int wheel_diameter_mm = 66){
  double circonference = 66e-3 * 3.14;
  double rounds = (double) wheel_ticks / (double) (gear_ratio * ticks_per_rounds);
  return rounds * circonference;
}

void Odom_tf2_broadcaster::encoder_left_callback(const std_msgs::Int32::ConstPtr& encoder_msg){
  current_time_left = ros::Time::now();

  double dt = (current_time_left - last_time_left).toSec();

  last_ticks_left = current_ticks_left;
  current_ticks_left = encoder_msg->data;

  d_left = ticks_to_meters(current_ticks_left - last_ticks_left);

  if (dt > 0){
    v_left = d_left / dt;
  } else {
    vx = 0;
  }

  last_time_left = current_time_left;
}

void Odom_tf2_broadcaster::encoder_right_callback(const std_msgs::Int32::ConstPtr& encoder_msg){
  current_time_right = ros::Time::now();

  double dt = (current_time_right - last_time_right).toSec();
  
  last_ticks_right = current_ticks_right;
  current_ticks_right = encoder_msg->data;

  d_right = ticks_to_meters(current_ticks_right - last_ticks_right);

  if (dt > 0){
    v_right =  d_right / dt;
  } else {
    vy = 0;
  }

  last_time_right = current_time_right;
}

Odom_tf2_broadcaster::Odom_tf2_broadcaster(void){
  x = 0.0;
  y = 0.0;
  th = 0.0;
  vx = 0.0;
  vy = 0.0;
  vth = 0.0;
  d_left = d_right = 0;
  v_left = v_right = 0;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  last_time_left = current_time_left = ros::Time::now();
  last_ticks_left = current_ticks_left = 0;

  last_time_right = current_time_right = ros::Time::now();
  last_ticks_right = current_ticks_right = 0;

  base_width_m = 180e-3;

}

// Odom_tf2_broadcaster::Odom_tf2_broadcaster(std::string topic, std::string frame_id, std::string child_frame_id){
//   x = 0.0;
//   y = 0.0;
//   th = 0.0;
//   vx = 0.1;
//   vy = -0.1;
//   vth = 0.1;
//   current_time = ros::Time::now();
//   last_time = ros::Time::now();

//   odom_trans.header.frame_id = frame_id;
//   odom_trans.child_frame_id = child_frame_id;
// }

Odom_tf2_broadcaster::~Odom_tf2_broadcaster(void){}

// void Odom_tf2_broadcaster::setup(ros::NodeHandle *nh, std::string topic, std::string frame_id, std::string child_frame_id){
void Odom_tf2_broadcaster::setup(ros::NodeHandle *nh, std::string topic_left, std::string topic_right, std::string frame_id, std::string child_frame_id) {
  nh_ = nh;
  
  odom_pub = nh_->advertise<nav_msgs::Odometry>("odom", 50);

  sub_odom_data_left = nh_->subscribe(topic_left.c_str(), 1000, &Odom_tf2_broadcaster::encoder_left_callback, this);
  sub_odom_data_right = nh_->subscribe(topic_right.c_str(), 1000, &Odom_tf2_broadcaster::encoder_right_callback, this);

  odom_trans.header.frame_id = frame_id;
  odom_trans.child_frame_id = child_frame_id;

  odom.header.frame_id = frame_id;
  odom.child_frame_id = child_frame_id;

}

void Odom_tf2_broadcaster::set_init_rotation(double roll, double pitch, double yaw){
  robot_init_orientation.setRPY(roll, pitch, yaw);
  th = yaw;
}

void Odom_tf2_broadcaster::set_rotation(double roll, double pitch, double yaw){
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);

  q *= robot_init_orientation; 
  q.normalize();
  odom_trans.transform.rotation.x = q.x();
  odom_trans.transform.rotation.y = q.y();
  odom_trans.transform.rotation.z = q.z();
  odom_trans.transform.rotation.w = q.w();
}

void Odom_tf2_broadcaster::set_rotation(tf2::Quaternion q){
  q *= robot_init_orientation;
  q.normalize();
  odom_trans.transform.rotation.x = q.x();
  odom_trans.transform.rotation.y = q.y();
  odom_trans.transform.rotation.z = q.z();
  odom_trans.transform.rotation.w = q.w();
}

void Odom_tf2_broadcaster::set_rotation(geometry_msgs::Quaternion q){
  tf2::Quaternion q_orig;
  tf2::convert(q , q_orig);

  q_orig *= robot_init_orientation;
  q_orig.normalize();

  tf2::convert(q_orig, q);

  odom_trans.transform.rotation.x = q.x;
  odom_trans.transform.rotation.y = q.y;
  odom_trans.transform.rotation.z = q.z;
  odom_trans.transform.rotation.w = q.w;
}

void Odom_tf2_broadcaster::set_translation(double x, double y, double z){
  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = z;

  this->x = x;
  this->y = y;
}

void Odom_tf2_broadcaster::set_translation(geometry_msgs::Vector3 translation){
  odom_trans.transform.translation = translation;
}

void Odom_tf2_broadcaster::loop(){
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();

    dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    double delta_d = (d_left + d_right) / 2.0 ;
    double delta_yaw = ( d_right - d_left ) / base_width_m ;

    vx = (v_left + v_right) / 2;
    vy = 0.0;
    vth = (v_right - v_left) / base_width_m;

    if(0 != delta_d){
      delta_x = delta_d * cos(delta_yaw);
      delta_y = -delta_d * sin(delta_yaw);

      x += ( cos( th ) * delta_x - sin( th ) * delta_y );
      y += ( sin( th ) * delta_x + cos( th ) * delta_y );
    }

    if (0 != delta_yaw){
      th += delta_yaw;
    }


    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    // geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    // odom_trans.header.frame_id = "odom";
    // odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    // nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    // odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    // odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
}