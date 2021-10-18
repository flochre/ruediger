#include <ros/ros.h>
#include <math.h>

#include "imu_tf2_broadcaster.hpp"
#include "odom_tf2_broadcaster.hpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "tf2_manager");
    ros::NodeHandle nh;

    Imu_tf2_broadcaster     tf2_imu;
    Odom_tf2_broadcaster    tf2_odom;

    tf2_imu.setup(&nh, "/serial/imu_data", "world", "imu_data");
    tf2_imu.set_init_rotation(0.0, 0.0, -M_PI/2);
    tf2_imu.set_translation(1.30, 0.75, 0);

    int counter = 0;
    boost::shared_ptr<std_msgs::Int32 const> sharedPtr;
    std_msgs::Int32 encoder_start_value;

    sharedPtr  = ros::topic::waitForMessage<std_msgs::Int32>("/serial/encoder_2", ros::Duration(1));
    counter++;
    while(NULL == sharedPtr && counter < 15){
        counter++;
        sharedPtr  = ros::topic::waitForMessage<std_msgs::Int32>("/serial/encoder_2", ros::Duration(1));
        ROS_WARN("No Encoder_2 received waiting");
    }

    if (sharedPtr != NULL) {
        encoder_start_value = *sharedPtr;
        tf2_odom.set_ticks_left(encoder_start_value.data);
    } else {
        ROS_ERROR("No Encoder_2 received");
    }

    counter = 0;
    sharedPtr = NULL;

    sharedPtr  = ros::topic::waitForMessage<std_msgs::Int32>("/serial/encoder_3", ros::Duration(1));
    counter++;
    while(NULL == sharedPtr && counter < 15){
        counter++;
        sharedPtr  = ros::topic::waitForMessage<std_msgs::Int32>("/serial/encoder_3", ros::Duration(1));
        ROS_WARN("No Encoder_3 received waiting..");
    }

    if (sharedPtr != NULL) {
        encoder_start_value = *sharedPtr;
        tf2_odom.set_ticks_right(encoder_start_value.data);
    } else {
        ROS_ERROR("No Encoder_3 received");
    }
        
    tf2_odom.setup(&nh, "/serial/encoder_2", "/serial/encoder_3", "world", "odom");
    tf2_odom.set_init_rotation(0.0, 0.0, -M_PI/2);
    tf2_odom.set_init_translation(1.30, 0.75);

    ros::Rate rate(12.0);
    while (nh.ok()){
        tf2_imu.loop();
        ros::spinOnce();

        tf2_odom.loop();
        ros::spinOnce();
        rate.sleep();
    }

};
