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
    if (abs(v_left) > 0.6) {
      v_left = (v_left/abs(v_left) * 0.6);
    }
  } else {
    v_left = 0;
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
    if (abs(v_right) > 0.6) {
      v_right = (v_right/abs(v_right) * 0.6);
    }
  } else {
    v_right = 0;
  }

  last_time_right = current_time_right;
}

Odom_tf2_broadcaster::Odom_tf2_broadcaster(void) : Tf2_broadcaster(){
  // robot_init_orientation.setRPY(0, 0, 0);
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

  // base_width_m = 180e-3; // 270e-3 292e-3 315e-3 327e-3
  base_width_m = 281e-3;

}

Odom_tf2_broadcaster::~Odom_tf2_broadcaster(void){}

void Odom_tf2_broadcaster::setup(ros::NodeHandle *nh, std::string topic_left, std::string topic_right, std::string frame_id, std::string child_frame_id) {
  nh_ = nh;
  
  odom_pub = nh_->advertise<nav_msgs::Odometry>("odom", 50);

  sub_odom_data_left = nh_->subscribe(topic_left.c_str(), 1000, &Odom_tf2_broadcaster::encoder_left_callback, this);
  sub_odom_data_right = nh_->subscribe(topic_right.c_str(), 1000, &Odom_tf2_broadcaster::encoder_right_callback, this);

  tf_stamped.header.frame_id = frame_id;
  tf_stamped.child_frame_id = child_frame_id;

  tf_stamped.header.frame_id = frame_id;
  tf_stamped.child_frame_id = child_frame_id;

}

void Odom_tf2_broadcaster::set_init_rotation(double roll, double pitch, double yaw){
  th = yaw;
}

void Odom_tf2_broadcaster::set_init_translation(double x, double y){
  this->x = x;
  this->y = y;
}

void Odom_tf2_broadcaster::set_ticks_left(int current_ticks){
  current_ticks_left = last_ticks_left = current_ticks;
}

void Odom_tf2_broadcaster::set_ticks_right(int current_ticks){
  current_ticks_right = last_ticks_right = current_ticks;
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
    // odom_trans.header.stamp = current_time;

    // odom_trans.header.frame_id = "odom";
    // odom_trans.child_frame_id = "base_link";

    // odom_trans.transform.translation.x = x;
    // odom_trans.transform.translation.y = y;
    // odom_trans.transform.translation.z = 0.0;
    this->set_translation(x, y, 0);

    // odom_trans.transform.rotation = odom_quat;
    this->set_rotation(odom_quat);

    //send the transform
    // odom_broadcaster.sendTransform(odom_trans);
    Tf2_broadcaster::loop();

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
