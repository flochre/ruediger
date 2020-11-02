#include "drive.hpp"

Drive::Drive(void){}

void Drive::setup(ros::NodeHandle *nh, char sub_vx_name[], char sub_vy_name[], char sub_vteta_name[], 
    ros::Subscriber<std_msgs::Int32>::CallbackT cb_vx, ros::Subscriber<std_msgs::Int32>::CallbackT cb_vy, ros::Subscriber<std_msgs::Int32>::CallbackT cb_vteta) {
    nh_ = nh;
  
    drive_vx = new ros::Subscriber<std_msgs::Int32>(sub_vx_name, cb_vx);
    nh_->subscribe(*drive_vx);
    drive_vy = new ros::Subscriber<std_msgs::Int32>(sub_vy_name, cb_vy);
    nh_->subscribe(*drive_vy);
    drive_vteta = new ros::Subscriber<std_msgs::Int32>(sub_vteta_name, cb_vteta);
    nh_->subscribe(*drive_vteta);
}

void Drive::loop() {

}