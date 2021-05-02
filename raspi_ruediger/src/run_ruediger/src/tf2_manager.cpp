#include <ros/ros.h>
#include "tf2_broadcaster.hpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "tf2_manager");
    ros::NodeHandle nh;

    Tf2_broadcaster tf2_imu;

    tf2_imu.setup(&nh, "/serial/imu_data", "world", "imu_data");
    tf2_imu.set_init_rotation(0.0, 0.0, -1.57);
    tf2_imu.set_translation(1.30, 0.75, 0);

    ros::Rate rate(10.0);
    while (nh.ok()){

        tf2_imu.loop();
        ros::spinOnce();
        rate.sleep();
    }

};
