#include "imu.hpp"

Imu::Imu(void) {
    timer = 0;
    // sensorPin = A9;
    sensorPin = mePort[6].s2;
    mpuInterrupt = false;
    dmpReady = false;
}

unsigned long Imu::read_timer(void){
    return timer;
}

void Imu::setup(ros::NodeHandle *nh, char topic_name[]){
    nh_ = nh;
  
    // Create publisher and advertise it!
    imu_pub = new ros::Publisher(topic_name, &imu_msgs);          
    nh_->advertise(*imu_pub);

    imu_msgs.header.frame_id = frame_id.c_str();

    #ifdef DEBUG
        Serial.begin(115200);
        while (!Serial);
    #endif

    // Init Gyroscope
    #ifndef MPU
    gyro = new MeGyro;
    gyro->begin();
    packetSize = 42;
    #endif

    #ifdef MPU
    Wire.begin();
    DEBUG_PRINTLN(F("Debug Started :)"));
    mpu.initialize();
    
    devStatus = mpu.dmpInitialize();
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } 
    #endif
}

void Imu::loop(void) {
    // DEBUG_PRINT(F("Cycle IMU:"));
    // DEBUG_PRINTLN(millis()-timer);
    timer = millis();

    #ifdef MPU
    mpu.resetFIFO();
    if (!dmpReady) return;

    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    DEBUG_PRINT(F("current FIFO count :"));
    DEBUG_PRINTLN(fifoCount);

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();

        DEBUG_PRINTLN(F("FIFO Overflow"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x01) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // #ifdef DEBUG
        // for (size_t i = 0; i < fifoCount; i++)
        // {
        //     DEBUG_PRINT(fifoBuffer[i]);
        // }
        // DEBUG_PRINTLN();
        // #endif
        
        
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        // fifoCount -= packetSize;
        // mpu.resetFIFO();
        // fifoCount = 0;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetGyro(ypr_rate, fifoBuffer);
        // mpu.dmpGetEuler(euler, &q);
        // mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

        imu_msgs.header.stamp = nh_->now();

        // //Assigning quaternion to IMU message and publishing the values
        imu_msgs.orientation.x = q.x;
        imu_msgs.orientation.y = q.y;
        imu_msgs.orientation.z = q.z;
        imu_msgs.orientation.w = q.w;

        // imu_msgs.angular_velocity.x = ypr[2]* 180/M_PI;
        // imu_msgs.angular_velocity.y = ypr[1]* 180/M_PI;
        // imu_msgs.angular_velocity.z = ypr[0]* 180/M_PI;

        imu_msgs.angular_velocity.x = ypr_rate[0];
        imu_msgs.angular_velocity.y = ypr_rate[1];
        imu_msgs.angular_velocity.z = ypr_rate[2];

        // imu_msgs.angular_velocity.x = euler[0]* 180/M_PI;
        // imu_msgs.angular_velocity.y = euler[1]* 180/M_PI;
        // imu_msgs.angular_velocity.z = euler[2]* 180/M_PI;

        // // uint8_t buffer[14];
        // // I2Cdev::readByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, buffer);
        // // imu_msgs.linear_acceleration.x = buffer[0];
        // // imu_msgs.linear_acceleration.y = buffer[1];
        // // I2Cdev::readByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, buffer);
        // // imu_msgs.linear_acceleration.z = buffer[0];

        // imu_msgs.linear_acceleration.x = aaReal.x / 16384.0 * 9.8 ;
        // imu_msgs.linear_acceleration.y = aaReal.y / 16384.0 * 9.8 ;
        // imu_msgs.linear_acceleration.z = aaReal.z / 16384.0 * 9.8;       

        // imu_msgs.linear_acceleration.x = aaReal.x ;
        // imu_msgs.linear_acceleration.y = aaReal.y ;
        // imu_msgs.linear_acceleration.z = aaReal.z ;       

        imu_msgs.linear_acceleration.x = aa.x / 16384.0 * 9.8 ;
        imu_msgs.linear_acceleration.y = aa.y / 16384.0 * 9.8 ;
        imu_msgs.linear_acceleration.z = aa.z / 16384.0 * 9.8;  
        
        // imu_msgs.linear_acceleration.x = gravity.x;
        // imu_msgs.linear_acceleration.y = gravity.y;
        // imu_msgs.linear_acceleration.z = gravity.z;

        imu_pub->publish(&imu_msgs);
    }
    #endif

    #ifndef MPU
    gyro->update();
    imu_msgs.header.stamp = nh_->now();


    gyro->getQuaternion(&q);
    imu_msgs.orientation.x = q.x;
    imu_msgs.orientation.y = q.y;
    imu_msgs.orientation.z = q.z;
    imu_msgs.orientation.w = q.w;

    gyro->getGravity(&gravity, &q);

    double covariance_factor = TIMER_IMU * 0.001 * 0.2 * 0.01 * 250;
    double covariance = (covariance_factor * M_PI / 180) * (covariance_factor * M_PI / 180);
    imu_msgs.orientation_covariance[0] = covariance;
    imu_msgs.orientation_covariance[1] = covariance;
    imu_msgs.orientation_covariance[2] = covariance;
    imu_msgs.orientation_covariance[3] = covariance;
    imu_msgs.orientation_covariance[4] = covariance;
    imu_msgs.orientation_covariance[5] = covariance;
    imu_msgs.orientation_covariance[6] = covariance;
    imu_msgs.orientation_covariance[7] = covariance;
    imu_msgs.orientation_covariance[8] = covariance;

    imu_msgs.angular_velocity.x = gyro->getGyroX();
    imu_msgs.angular_velocity.y = gyro->getGyroY();
    imu_msgs.angular_velocity.z = gyro->getGyroZ();

    // imu_msgs.angular_velocity.x = gyro->getAngleX() * 180 / M_PI;
    // imu_msgs.angular_velocity.y = gyro->getAngleY() * 180 / M_PI;
    // imu_msgs.angular_velocity.z = gyro->getAngleZ() * 180 / M_PI;

    imu_msgs.linear_acceleration.x = gyro->getAccX();
    imu_msgs.linear_acceleration.y = gyro->getAccY();
    imu_msgs.linear_acceleration.z = gyro->getAccZ();

    // imu_msgs.linear_acceleration.x = gyro->getAccX() - gravity.x * 9.8;
    // imu_msgs.linear_acceleration.y = gyro->getAccY() - gravity.y * 9.8;
    // imu_msgs.linear_acceleration.z = gyro->getAccZ() - gravity.z * 9.8;

    // imu_msgs.linear_acceleration.x = gravity.x;
    // imu_msgs.linear_acceleration.y = gravity.y;
    // imu_msgs.linear_acceleration.z = gravity.z;
      
    // DEBUG_PRINT(F("Inside IMU loop without publishing:"));
    // DEBUG_PRINTLN(millis()-timer);

    imu_pub->publish(&imu_msgs);
    #endif

    // DEBUG_PRINT(F("Loop time cycle :"));
    // DEBUG_PRINTLN(millis()-timer);

}
