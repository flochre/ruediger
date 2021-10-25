#include <ros/ros.h>
#include <math.h>

#include "imu_tf2_broadcaster.hpp"
#include "odom_tf2_broadcaster.hpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "tf2_manager");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    Imu_tf2_broadcaster     tf2_imu;
    Odom_tf2_broadcaster    tf2_odom;

    std::string imu_topic;
    std::string imu_frame_id;
    std::string imu_child_frame_id;

    nh_priv.param<std::string>("imu_topic", imu_topic, "/serial/imu_data");
    nh_priv.param<std::string>("imu_frame_id", imu_frame_id, "/world");
    nh_priv.param<std::string>("imu_child_frame_id", imu_child_frame_id, "/imu_data");

    std::string odom_mot_left_topic;
    std::string odom_mot_right_topic;
    std::string odom_frame_id;
    std::string odom_child_frame_id;

    nh_priv.param<std::string>("odom_mot_left_topic", odom_mot_left_topic, "/serial/encoder_2");
    nh_priv.param<std::string>("odom_mot_right_topic", odom_mot_right_topic, "/serial/encoder_3");
    nh_priv.param<std::string>("odom_frame_id", odom_frame_id, "/world");
    nh_priv.param<std::string>("odom_child_frame_id", odom_child_frame_id, "/odom");

    double init_x, init_y, init_z, init_roll, init_pitch, init_yaw;

    nh_priv.param<double>("init_x", init_x, 0.0);
    nh_priv.param<double>("init_y", init_y, 0.0);
    nh_priv.param<double>("init_z", init_z, 0.0);
    nh_priv.param<double>("init_roll", init_roll, 0.0);
    nh_priv.param<double>("init_pitch", init_pitch, 0.0);
    nh_priv.param<double>("init_yaw", init_yaw, 0.0);

    // tf2_imu.setup(&nh, "/serial/imu_data", "world", "imu_data");
    tf2_imu.setup(&nh, imu_topic, imu_frame_id, imu_child_frame_id);
    // tf2_imu.set_init_rotation(0.0, 0.0, -M_PI/2);
    tf2_imu.set_init_rotation(init_roll, init_pitch, init_yaw);
    // tf2_imu.set_translation(1.30, 0.75, 0);
    tf2_imu.set_translation(init_x, init_y, init_z);

    int counter = 0;
    boost::shared_ptr<std_msgs::Int32 const> sharedPtr;
    std_msgs::Int32 encoder_start_value;

    sharedPtr  = ros::topic::waitForMessage<std_msgs::Int32>(odom_mot_left_topic, ros::Duration(1));
    counter++;
    while(NULL == sharedPtr && counter < 15){
        counter++;
        sharedPtr  = ros::topic::waitForMessage<std_msgs::Int32>(odom_mot_left_topic, ros::Duration(1));
        ROS_WARN("No Encoder_Left infos received yet waiting..");
    }

    if (sharedPtr != NULL) {
        encoder_start_value = *sharedPtr;
        tf2_odom.set_ticks_left(encoder_start_value.data);
    } else {
        ROS_ERROR("No Encoder_Left received", odom_mot_left_topic);
    }

    counter = 0;
    sharedPtr = NULL;

    sharedPtr  = ros::topic::waitForMessage<std_msgs::Int32>(odom_mot_right_topic, ros::Duration(1));
    counter++;
    while(NULL == sharedPtr && counter < 15){
        counter++;
        sharedPtr  = ros::topic::waitForMessage<std_msgs::Int32>(odom_mot_right_topic, ros::Duration(1));
        ROS_WARN("No Encoder_Right received yet waiting..");
    }

    if (sharedPtr != NULL) {
        encoder_start_value = *sharedPtr;
        tf2_odom.set_ticks_right(encoder_start_value.data);
    } else {
        ROS_ERROR("No Encoder_Right received");
    }
        
    // tf2_odom.setup(&nh, "/serial/encoder_2", "/serial/encoder_3", "world", "odom");
    tf2_odom.setup(&nh, odom_mot_left_topic, odom_mot_right_topic, odom_frame_id, odom_child_frame_id);
    // tf2_odom.set_init_rotation(0.0, 0.0, -M_PI/2);
    tf2_odom.set_init_rotation(init_roll, init_pitch, init_yaw);
    // tf2_odom.set_init_translation(1.30, 0.75);
    tf2_odom.set_init_translation(init_x, init_y);

    ros::Rate rate(12.0);
    while (nh.ok()){
        tf2_imu.loop();
        ros::spinOnce();

        tf2_odom.loop();
        ros::spinOnce();
        rate.sleep();
    }

};
