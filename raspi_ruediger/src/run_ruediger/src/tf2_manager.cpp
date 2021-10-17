#include <ros/ros.h>
#include "imu_tf2_broadcaster.hpp"
#include "odom_tf2_broadcaster.hpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "tf2_manager");
    ros::NodeHandle nh;

    Imu_tf2_broadcaster     tf2_imu;
    Odom_tf2_broadcaster    tf2_odom;

    tf2_imu.setup(&nh, "/serial/imu_data", "world", "imu_data");
    tf2_imu.set_init_rotation(0.0, 0.0, -1.57);
    tf2_imu.set_translation(1.30, 0.75, 0);

    // tf2_odom.setup(&nh, "/serial/imu_data", "world", "imu_data");
    // tf2_odom.setup(&nh, "/serial/encoder_2", "/serial/encoder_3", "odom", "world");
    tf2_odom.setup(&nh, "/serial/encoder_2", "/serial/encoder_3", "world", "odom");
    tf2_odom.set_init_rotation(0.0, 0.0, -1.57);
    tf2_odom.set_init_translation(1.30, 0.75);

    ros::Rate rate(12.0);
    while (nh.ok()){

        // tf2_imu.loop();
        ros::spinOnce();

        tf2_odom.loop();
        ros::spinOnce();
        rate.sleep();
    }

};
