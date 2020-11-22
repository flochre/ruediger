#include "drive.hpp"

Drive::Drive(void){}

void Drive::setup(ros::NodeHandle *nh, char sub_cmd_vel[], ros::Subscriber<geometry_msgs::Twist>::CallbackT cb_cmd_vel) {
    nh_ = nh;

    cmd_vel = new ros::Subscriber<geometry_msgs::Twist>(sub_cmd_vel, cb_cmd_vel);
    nh_->subscribe(*cmd_vel);
}

void Drive::loop() {

}