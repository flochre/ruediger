#include "drive.hpp"

Drive::Drive(void){}

// void Drive::setup(ros::NodeHandle *nh, char sub_vx_name[], char sub_vy_name[], char sub_vteta_name[], 
void Drive::setup(ros::NodeHandle *nh, char sub_cmd_vel[], ros::Subscriber<geometry_msgs::Twist>::CallbackT cb_cmd_vel) {
    nh_ = nh;
  
    // drive_vx = new ros::Subscriber<std_msgs::Int32>(sub_vx_name, cb_vx);
    // nh_->subscribe(*drive_vx);
    // drive_vy = new ros::Subscriber<std_msgs::Int32>(sub_vy_name, cb_vy);
    // nh_->subscribe(*drive_vy);
    // drive_vteta = new ros::Subscriber<std_msgs::Int32>(sub_vteta_name, cb_vteta);
    // nh_->subscribe(*drive_vteta);

    cmd_vel = new ros::Subscriber<geometry_msgs::Twist>(sub_cmd_vel, cb_cmd_vel);
    nh_->subscribe(*cmd_vel);
}

void Drive::loop() {

}