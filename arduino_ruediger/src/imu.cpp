#include "imu.hpp"

Imu::Imu(void) : MePort(0x6){
  timer = millis();
  device_address = MPU6050_DEFAULT_ADDRESS;

  #ifdef ME_PORT_DEFINED
  _AD0 = s1;
  _INT = s2;
  #endif // ME_PORT_DEFINED

  // Set _AD0 to the ground
  // https://www.i2cdevlib.com/forums/topic/414-freezing-problem/
  MePort::dWrite1(LOW);
}

void Imu::begin(uint8_t accel_config, uint8_t gyro_config) {
  aSensitivity = set_aSensitivity(accel_config);
  gSensitivity = set_gSensitivity(gyro_config);

  set_gyro_angles = false;
  angle_pitch = 0, angle_yaw = 0, angle_roll = 0;
  angle_pitch_acc = 0, angle_yaw_acc = 0, angle_roll_acc = 0;


  Wire.begin();

  delay(200);
  //close the sleep mode
  I2Cdev::writeByte(device_address, MPU6050_RA_PWR_MGMT_1, 0x00);

  delay(100);
  //configurate the digital low pass filter
  I2Cdev::writeByte(device_address, MPU6050_RA_CONFIG, 0x01);

  //set the accel scale
  I2Cdev::writeBits(device_address, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, accel_config);
  //set the gyro scale
  I2Cdev::writeBits(device_address, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, gyro_config);

  delay(100);
  calibrate(200);

}

void Imu::calibrate(uint16_t calibration_iterations){
  acc_x_cal = 0, acc_y_cal = 0, acc_z_cal = 0;
  gyro_x_cal = 0, gyro_y_cal = 0, gyro_z_cal = 0;

  for (size_t i = 0; i < calibration_iterations; i++) {
    read_mpu_6050_data();
    acc_x_cal += acc_x;
    acc_y_cal += acc_y;
    acc_z_cal += acc_z;

    gyro_x_cal += gyro_x;
    gyro_y_cal += gyro_y;
    gyro_z_cal += gyro_z;
    delay(3);
  }
  
  acc_x_cal /= calibration_iterations;
  acc_y_cal /= calibration_iterations;
  acc_z_cal /= calibration_iterations;
  
  gyro_x_cal /= calibration_iterations;
  gyro_y_cal /= calibration_iterations;
  gyro_z_cal /= calibration_iterations;

}

void Imu::read_mpu_6050_data(void){
  /* read imu data */
  if(!I2Cdev::readBytes(device_address, MPU6050_RA_ACCEL_XOUT_H, I2C_BUFFER_SIZE, i2cData)) {
    return;
  }

  acc_x   = ( (i2cData[0] << 8) | i2cData[1] );
  acc_y   = ( (i2cData[2] << 8) | i2cData[3] );
  acc_z   = ( (i2cData[4] << 8) | i2cData[5] );

  temperature = ( (i2cData[6] << 8) | i2cData[7] );

  gyro_x  = ( (i2cData[8] << 8)  | i2cData[9]  );
  gyro_y  = ( (i2cData[10] << 8) | i2cData[11] );
  gyro_z  = ( (i2cData[12] << 8) | i2cData[13] );
}

double Imu::get_aSensitivity(void){
  return gSensitivity;
}

double Imu::set_aSensitivity(uint8_t accel_config){
  switch (accel_config) {
    case MPU6050_ACCEL_FS_2:
      aSensitivity = 16384.0;
      break;
    
    case MPU6050_ACCEL_FS_4:
      aSensitivity = 8192.0;
      break;

    case MPU6050_ACCEL_FS_8:
      aSensitivity = 4096.0;
      break;

    case MPU6050_ACCEL_FS_16:
      aSensitivity = 2048.0;
      break;

    default:
      aSensitivity = 16384.0;
      break;
  }

  aSensitivity_si = aSensitivity / 9.8; // LSB/g to LSB/ms-2 

  return aSensitivity;
}

double Imu::get_gSensitivity(void){
  return gSensitivity;
}

double Imu::set_gSensitivity(uint8_t gyro_config){
  switch (gyro_config)
  {
    case MPU6050_GYRO_FS_250:
      gSensitivity = 131.0;
      break;
    
    case MPU6050_GYRO_FS_500:
      gSensitivity = 65.5;
      break;

    case MPU6050_GYRO_FS_1000:
      gSensitivity = 32.8;
      break;

    case MPU6050_GYRO_FS_2000:
      gSensitivity = 16.4;
      break;

    default:
      gSensitivity = 65.5;
      break;
  }

  gSensitivity_si = gSensitivity * 180 / M_PI; // LSB/°/s to LSB / rad / s 

  return gSensitivity;
}

unsigned long Imu::read_timer(void){
  return timer;
}

static inline int8_t sgn(int val) {
  if (val < 0) return -1;
  if (val==0) return 0;
  return 1;
}

void Imu::update(float tau) {
  static unsigned long	last_time = micros();
  double dt, filter_coefficient;
  
  dt = (double)(micros() - last_time) * 0.000001; // dt in secondes
  last_time = micros();
  read_mpu_6050_data();

  gyro_x -= gyro_x_cal;   //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;   //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;   //Subtract the offset calibration value from the raw gyro_z value

  gyro_x /= gSensitivity_si;
  gyro_y /= gSensitivity_si;
  gyro_z /= gSensitivity_si;

  // angle_yaw += gyro_z / gSensitivity_si * dt;
  angle_yaw += gyro_z * dt;
  angle_yaw = angle_yaw - (2 * M_PI) * floor(angle_yaw / (2 * M_PI));
  // to get yaw between [-M_PI; M_PI]
  if(angle_yaw > M_PI){
      angle_yaw -= 2 * M_PI;
  } 

  if (acc_z > 0) {
      angle_pitch += gyro_y * dt;
      angle_roll  += gyro_x * dt;
  } else {
      angle_pitch -= gyro_y * dt;
      angle_roll  -= gyro_x * dt;
  }

  angle_pitch -= angle_roll * sin(gyro_z * dt);
  angle_roll  += angle_pitch * sin(gyro_z * dt);

  // //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));
  angle_pitch_acc = -asin((float)acc_x/acc_total_vector);      // Calculate the pitch angle based on acceleration
  angle_roll_acc  =  asin((float)acc_y/acc_total_vector);      // Calculate the roll angle based on acceleration
  
  // Set acceleration values
  if(set_gyro_angles){
    if (0x1 == DELETE_GRAVITY) {
      acc_x -= acc_x_cal + gravity_calib[0] ;
      acc_y -= acc_y_cal + gravity_calib[1] ;
      acc_z -= acc_z_cal + gravity_calib[2] ; 
    }

    // if (0x0 == DELETE_GRAVITY) {
    //   acc_x -= acc_x_cal;
    //   acc_y -= acc_y_cal;
    //   acc_z -= acc_z_cal; 
    // }
  }

  acc_x /= aSensitivity_si;
  acc_y /= aSensitivity_si;
  acc_z /= aSensitivity_si;

  /*
      complementary filter
      set 0.5sec = tau = dt * A / (1 - A)
      so A = tau / (tau + dt)
  */

  // float tau = 0.5;
  filter_coefficient = tau / (tau + dt);
  filter_coefficient = 0.9996;

  if(set_gyro_angles){
      angle_roll = angle_roll * filter_coefficient + angle_roll_acc * (1 - filter_coefficient);
      angle_pitch = angle_pitch * filter_coefficient + angle_pitch_acc * (1 - filter_coefficient);
  } else {                                                                
      //At first start
      angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
      angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
      angle_yaw = 0;
      set_gyro_angles = true;

      // Calculate init quaternion, to get the init gravitation
      get_quaternion(&imu_msgs.orientation);
      get_gravity(gravity_calib, &imu_msgs.orientation);
  }

  // filter_coefficient = 0.8;
  filter_coefficient = tau / (tau + dt);
  angle_pitch_output  = angle_pitch_output * filter_coefficient + angle_pitch * (1 - filter_coefficient);   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output   = angle_roll_output * filter_coefficient + angle_roll * (1 - filter_coefficient);
  // angle_yaw_output    = sgn(angle_yaw) * (abs(angle_yaw_output) * filter_coefficient + abs(angle_yaw) * (1 - filter_coefficient));
  angle_yaw_output    = angle_yaw;
}

uint8_t Imu::get_quaternion(geometry_msgs::Quaternion *q){
  return get_quaternion(q, angle_yaw_output, angle_pitch_output, angle_roll_output);
}

uint8_t Imu::get_quaternion(geometry_msgs::Quaternion *q, double yaw, double pitch, double roll){
  // Abbreviations for the various angular functions
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  // Quaternion q;
  q->w = cr * cp * cy + sr * sp * sy;
  q->x = sr * cp * cy - cr * sp * sy;
  q->y = cr * sp * cy + sr * cp * sy;
  q->z = cr * cp * sy - sr * sp * cy;
  return 0;
}

uint8_t Imu::get_gravity(float *v, geometry_msgs::Quaternion *q) {
  v[0] = 2 * (q->x * q->z - q->w * q->y);
  v[1] = 2 * (q->w * q->x + q->y * q->z);
  v[2] = q->w * q->w - q->x * q->x - q->y * q->y + q->z * q->z;
  return 0;
}

// void Imu::setup(ros::NodeHandle *nh, char topic_name[], char frame_id[]){
void Imu::setup(ros::NodeHandle *nh, char topic_name[]){
  nh_ = nh;

  // Create publisher and advertise it!
  imu_pub = new ros::Publisher(topic_name, &imu_msgs);          
  nh_->advertise(*imu_pub);

  // imu_msgs.header.frame_id = frame_id;
  imu_msgs.header.frame_id = topic_name;


  // Init Gyroscope
  begin(MPU6050_ACCEL_FS_2, MPU6050_GYRO_FS_500);
}

void Imu::loop(void) {
  timer = millis();

  update();
  imu_msgs.header.stamp = nh_->now();

  get_quaternion(&imu_msgs.orientation);

  get_gravity(gravity, &imu_msgs.orientation);

  // double covariance_factor = TIMER_IMU * 0.001 * 0.2 * 0.01 * 250;
  // double covariance = (covariance_factor * M_PI / 180) * (covariance_factor * M_PI / 180);
  // imu_msgs.orientation_covariance[0] = covariance;
  // imu_msgs.orientation_covariance[1] = covariance;
  // imu_msgs.orientation_covariance[2] = covariance;
  // imu_msgs.orientation_covariance[3] = covariance;
  // imu_msgs.orientation_covariance[4] = covariance;
  // imu_msgs.orientation_covariance[5] = covariance;
  // imu_msgs.orientation_covariance[6] = covariance;
  // imu_msgs.orientation_covariance[7] = covariance;
  // imu_msgs.orientation_covariance[8] = covariance;

  // imu_msgs.angular_velocity.x = angle_roll_output * 180 / M_PI;
  // imu_msgs.angular_velocity.y = angle_pitch_output * 180 / M_PI;
  // imu_msgs.angular_velocity.z = angle_yaw_output * 180 / M_PI;

  imu_msgs.angular_velocity.x = gyro_x;
  imu_msgs.angular_velocity.y = gyro_y;
  imu_msgs.angular_velocity.z = gyro_z;

  // imu_msgs.linear_acceleration.x = gravity[0];
  // imu_msgs.linear_acceleration.y = gravity[1];
  // imu_msgs.linear_acceleration.z = gravity[2];

  imu_msgs.linear_acceleration.x = acc_x;
  imu_msgs.linear_acceleration.y = acc_y;
  imu_msgs.linear_acceleration.z = acc_z;

  imu_pub->publish(&imu_msgs);
}