#include "imu.hpp"

Imu::Imu(void) {
}

void Imu::setup(ros::NodeHandle *nh, char topic_name[]){
    nh_ = nh;
  
    // Create publisher and advertise it!
    imu_pub = new ros::Publisher(topic_name, &imu_msgs);          
    nh_->advertise(*imu_pub);

    // Init Gyroscope
    gyro = new MeGyro;
    gyro->begin();

    packetSize = 42;
}

void Imu::loop(void)     {
    
    gyro->update();
    q = gyro->GetQuaternion();

    //Assigning quaternion to IMU message and publishing the values
    orient.x = q.x;
    orient.y = q.y;
    orient.z = q.z;
    orient.w = q.w;
    imu_msgs.header.frame_id = frameid;
    imu_msgs.orientation = orient;
    imu_pub->publish(&imu_msgs);

}
