#ifndef IMU_HPP
#define IMU_HPP

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_Reduced.h"

#include <ros.h>

//These are the headers to access IMU-ROS message header, quaternion message header and TF broadcaster
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>

#define TIMER_IMU 52      // The MPU is geting data every 4ms a multiple of 4 is better
#define DELETE_GRAVITY 0x0  // Use 1 to substract gravity use 
#define I2C_BUFFER_SIZE 14

class Imu {
  //  private: 
    unsigned long timer;

    // Creating handlers of Node, IMU message, quaternion and ROS publisher.
    ros::NodeHandle *nh_;
    ros::Publisher *imu_pub;

    sensor_msgs::Imu imu_msgs;

    double  aSensitivity, aSensitivity_si; /* for 2g, check data sheet AFS_SEL = 0 Register 28 (0x1c) */
    double  gSensitivity, gSensitivity_si; /* for 500 deg/s, check data sheet */

    boolean set_gyro_angles;
    double acc_x, acc_y, acc_z, acc_total_vector;
    long acc_x_cal, acc_y_cal, acc_z_cal;
    double temperature;
    double gyro_x, gyro_y, gyro_z;
    long gyro_x_cal, gyro_y_cal, gyro_z_cal;
    double angle_pitch, angle_yaw, angle_roll;
    double angle_pitch_output, angle_yaw_output, angle_roll_output;
    double angle_pitch_acc, angle_yaw_acc, angle_roll_acc;

    float gravity_calib[3];
    float gravity[3];       // [x, y, z]            gravity vector

    uint8_t i2cData[I2C_BUFFER_SIZE];
    uint8_t device_address;

    // String frame_id = "/base_link";
    // char frame_id[] = "/base_link";

    void begin(uint8_t accel_config = MPU6050_ACCEL_FS_2, uint8_t gyro_config = MPU6050_GYRO_FS_500);
    void calibrate(uint16_t calibration_iterations = 200);
    
    void read_mpu_6050_data(void);

    double set_aSensitivity(uint8_t accel_config);
    double set_gSensitivity(uint8_t gyro_config);

    void update(float tau = 0.5);

  public:
    Imu(void);
    unsigned long read_timer(void);

    double get_aSensitivity(void);
    double get_gSensitivity(void);

    uint8_t get_quaternion(geometry_msgs::Quaternion *q);
    uint8_t get_quaternion(geometry_msgs::Quaternion *q, double yaw, double pitch, double roll);
    uint8_t get_gravity(float *v, geometry_msgs::Quaternion *q);

    // void setup(ros::NodeHandle *nh, char topic_name[] = "imu_data", char frame_id[] = "base_link");
    void setup(ros::NodeHandle *nh, char topic_name[] = "imu_data");
    void loop(void);
};

#endif // IMU_HPP