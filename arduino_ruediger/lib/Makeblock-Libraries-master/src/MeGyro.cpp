/**
 * \par Copyright (C), 2012-2016, MakeBlock
 * \class   MeGyro
 * \brief   Driver for MeGyro module.
 * @file    MeGyro.cpp
 * @author  MakeBlock
 * @version V1.0.5
 * @date    2018/01/03
 * @brief   Driver for MeGyro module.
 *
 * \par Copyright
 * This software is Copyright (C), 2012-2016, MakeBlock. Use is subject to license \n
 * conditions. The main licensing options available are GPL V2 or Commercial: \n
 *
 * \par Open Source Licensing GPL V2
 * This is the appropriate option if you want to share the source code of your \n
 * application with everyone you distribute it to, and you also want to give them \n
 * the right to share who uses it. If you wish to use this software under Open \n
 * Source Licensing, you must contribute all your source code to the open source \n
 * community in accordance with the GPL Version 2 when your application is \n
 * distributed. See http://www.gnu.org/copyleft/gpl.html
 *
 * \par Description
 * This file is a drive for MeGyro module, It supports MeGyro V1.0 device provided
 * by MakeBlock.
 *
 * \par Method List:
 *
 *    1. void MeGyro::setpin(uint8_t AD0, uint8_t INT)
 *    2. void MeGyro::begin(void)
 *    3. void MeGyro::update(void)
 *    4. void MeGyro::fast_update(void)
 *    5. uint8_t MeGyro::getDevAddr(void)
 *    6. double MeGyro::getAngleX(void)
 *    7. double MeGyro::getAngleY(void)
 *    8. double MeGyro::getAngleZ(void)
 *    9. double MeGyro::getGyroX(void)
 *    10. double MeGyro::getGyroY(void)
 *
 * \par History:
 * <pre>
 * `<Author>`         `<Time>`        `<Version>`        `<Descr>`
 *  Lawrence         2015/09/02          1.0.0         rebuild the old lib.
 *  Lawrence         2015/09/10          1.0.1         Added some comments and macros.
 *  Mark Yan         2016/03/09          1.0.2         Add function fast_update.
 *  Mark Yan         2016/03/09          1.0.3         Add function getGyroX and getGyroY.
 *  Leo lu           2017/04/27          1.0.4         fix issue of z-axis output double. getAngle function just return, do not call update anymore.
 *  Mark Yan         2018/01/03          1.0.5         Adjust the initialization sequence to optimize the Z-axis drift.
 * </pre>
 *
 * @example MeGyroTest.ino
 */

/* Includes ------------------------------------------------------------------*/
#include "MeGyro.h"

/* Private functions ---------------------------------------------------------*/
#ifdef ME_PORT_DEFINED
/**
 * Alternate Constructor which can call your own function to map the MeGyro to arduino port,
 * no pins are used or initialized here
 */
MeGyro::MeGyro(void) : MePort(6)
{
  Device_Address = GYRO_DEFAULT_ADDRESS;
}

/**
 * Alternate Constructor which can call your own function to map the MeGyro to arduino port,
 * no pins are used or initialized here, but PWM frequency set to 976 Hz
 * \param[in]
 *   port - RJ25 port from PORT_1 to M2
 */
MeGyro::MeGyro(uint8_t port) : MePort(port)
{
  Device_Address = GYRO_DEFAULT_ADDRESS;
}

/**
 * Alternate Constructor which can call your own function to map the MeGyro to arduino port
 * and change the i2c device address
 * no pins are used or initialized here, but PWM frequency set to 976 Hz
 * \param[in]
 *   port - RJ25 port from PORT_1 to M2
 * \param[in]
 *   address - the i2c address you want to set
 */
MeGyro::MeGyro(uint8_t port, uint8_t address) : MePort(port)
{
  Device_Address = address;
}
#else  // ME_PORT_DEFINED
/**
 * Alternate Constructor which can call your own function to map the _AD0 and _INT to arduino port,
 * no pins are used or initialized here
 * \param[in]
 *   _AD0 - arduino gpio number
 * \param[in]
 *   _INT - arduino gpio number
 */
MeGyro::MeGyro(uint8_t AD0, uint8_t INT)
{
  Device_Address = GYRO_DEFAULT_ADDRESS;
  _AD0 = AD0;
  _INT = INT;
}

/**
 * Alternate Constructor which can call your own function to map the _AD0 and _INT to arduino port
 * and change the i2c device address, no pins are used or initialized here
 * \param[in]
 *   _AD0 - arduino gpio number
 * \param[in]
 *   _INT - arduino gpio number
 * \param[in]
 *   address - the i2c address you want to set
 */
MeGyro::MeGyro(uint8_t AD0, uint8_t INT, uint8_t address)
{
  Device_Address = address;
  _AD0 = AD0;
  _INT = INT;
}
#endif // ME_PORT_DEFINED

/**
 * \par Function
 *   setpin
 * \par Description
 *   Set the PIN of the button module.
 * \param[in]
 *   AD0 - pin mapping for arduino
 * \param[in]
 *   INT - pin mapping for arduino
 * \par Output
 *   None
 * \return
 *   None.
 * \par Others
 *   Set global variable _AD0, _INT, s1 and s2
 */
void MeGyro::setpin(uint8_t AD0, uint8_t INT)
{
  _AD0 = AD0;
  _INT = INT;
#ifdef ME_PORT_DEFINED
  s1 = AD0;
  s2 = INT;
#endif // ME_PORT_DEFINED
}

/**
 * \par Function
 *   begin
 * \par Description
 *   Initialize the MeGyro.
 * \param[in]
 *   None
 * \par Output
 *   None
 * \return
 *   None
 * \par Others
 *   You can check the MPU6050 datasheet for the registor address.
 */
// void MeGyro::begin(void)
// {
//   gSensitivity = 65.5; //for 500 deg/s, check data sheet
//   // gSensitivity = 16.4; //for 2000 deg/s, check data sheet FS_SEL = 3 
//   aSensitivity = 16384.0; // for 2g, check data sheet AFS_SEL = 0 Register 28 (0x1c)
//   gx = 0;
//   gy = 0;
//   gz = 0;
//   gyrX = 0;
//   gyrY = 0;
//   gyrZ = 0;
//   accX = 0;
//   accY = 0;
//   accZ = 0;
//   accXoffs = 0;
//   accYoffs = 0;
//   accZoffs = 0;
//   gyrXoffs = 0;
//   gyrYoffs = 0;
//   gyrZoffs = 0;
//   last_time = 0;
//   Wire.begin();
//   // Wire.setClock(400000);
//   delay(200);
//   writeReg(0x6b, 0x00);//close the sleep mode
//   delay(100);
//   writeReg(0x1a, 0x01);//configurate the digital low pass filter
//   writeReg(0x1b, 0x08);//set the gyro scale to 500 deg/s -> deviding by 65.5
//   // writeReg(0x1b, 0x18);//set the gyro scale to 2000 deg/s MPU6050_GYRO_FS_2000 -> deviding by 16.6
//   delay(100);
//   deviceCalibration();
// }

// setFullScaleGyroRange(MPU6050_GYRO_FS_250);
// setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

double MeGyro::get_aSensitivity(void){
  return gSensitivity;
}
double MeGyro::set_aSensitivity(uint8_t accel_config){
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

double MeGyro::get_gSensitivity(void){
  return gSensitivity;
}
double MeGyro::set_gSensitivity(uint8_t gyro_config){
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

void MeGyro::begin(uint8_t accel_config, uint8_t gyro_config) {
  aSensitivity = set_aSensitivity(accel_config);
  gSensitivity = set_gSensitivity(gyro_config);

  DEBUG_PRINTLN(F("Calibrate IMU :)"));
  DEBUG_PRINTLN(aSensitivity);
  DEBUG_PRINTLN(aSensitivity_si);
  DEBUG_PRINTLN(gSensitivity);
  DEBUG_PRINTLN(gSensitivity_si);

  gx = 0, gy = 0, gz = 0;
  gyrX = 0, gyrY = 0, gyrZ = 0;
  accX = 0, accY = 0, accZ = 0;

  set_gyro_angles = false;
  angle_pitch = 0, angle_yaw = 0, angle_roll = 0;
  angle_pitch_output = 0, angle_yaw_output = 0, angle_roll_output = 0;
  angle_pitch_acc = 0, angle_yaw_acc = 0, angle_roll_acc = 0;

  // accXoffs = 0, accYoffs = 0, accZoffs = 0;
  // gyrXoffs = 0, gyrYoffs = 0, gyrZoffs = 0;

  // last_time = 0;

  Wire.begin();
  // Wire.setClock(400000);
  delay(200);
  writeReg(0x6b, 0x00); //close the sleep mode
  delay(100);
  writeReg(0x1a, 0x01);//configurate the digital low pass filter
  
  //set the accel scale
  I2Cdev::writeBits(Device_Address, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, accel_config);
  //set the gyro scale
  I2Cdev::writeBits(Device_Address, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, gyro_config);
  
  #ifdef DEBUG
  readData(MPU6050_RA_GYRO_CONFIG, i2cData, 2);
  DEBUG_PRINTLN(i2cData[1]);
  DEBUG_PRINTLN(i2cData[0]);
  #endif
  delay(100);
  deviceCalibration(200);

}

/**
 * \par Function
 *   update
 * \par Description
 *   Update some calculated angle values to the variable.
 * \param[in]
 *   None
 * \par Output
 *   None
 * \return
 *   None
 * \par Others
 *   The angle values are calculated by complementary filter.
 *   The time constant of filter is set to 0.5 second, but period dt is not a constant, 
 *   so the filter coefficient will be calculated dynamically.
 */
void MeGyro::update(void) {
  static unsigned long	last_time = 0;
  double dt, filter_coefficient;
  
  dt = (double)(millis() - last_time) / 1000; // dt in secondes
  last_time = millis();
  read_mpu_6050_data();

  gyro_x -= gyro_x_cal;   //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;   //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;   //Subtract the offset calibration value from the raw gyro_z value

  gyro_x /= gSensitivity_si;
  gyro_y /= gSensitivity_si;
  gyro_z /= gSensitivity_si;

  // angle_yaw += gyro_z / gSensitivity_si * dt;
  angle_yaw += gyro_z * dt;
  // angle_yaw = angle_yaw - (2 * M_PI) * floor(angle_yaw / (2 * M_PI));
  // to get yaw between [-M_PI; M_PI]
  if(angle_yaw > M_PI) angle_yaw -= 2 * M_PI;

  if (acc_z > 0) {
    angle_pitch += gyro_y * dt;
    angle_roll  += gyro_x * dt;
  } else {
    angle_pitch -= gyro_y * dt;
    angle_roll  -= gyro_x * dt;
  }

  // angle_pitch -= angle_roll * sin(angle_yaw);
  // angle_roll  += angle_pitch * sin(angle_yaw);

  // //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));
  angle_pitch_acc = -asin((float)acc_x/acc_total_vector);      // Calculate the pitch angle based on acceleration
  angle_roll_acc  =  asin((float)acc_y/acc_total_vector);      // Calculate the roll angle based on acceleration
  
  // Set acceleration values
  if(set_gyro_angles){
    acc_x -= acc_x_cal + gravity_calib.x ;
    acc_y -= acc_y_cal + gravity_calib.y ;
    acc_z -= acc_z_cal + gravity_calib.z ; 
  }

  acc_x /= aSensitivity_si;
  acc_y /= aSensitivity_si;
  acc_z /= aSensitivity_si;

  /*
    complementary filter
    set 0.5sec = tau = dt * A / (1 - A)
    so A = tau / (tau + dt)
  */

  float tau = 0.5;
  filter_coefficient = tau / (tau + dt);
  filter_coefficient = 0.9996;

  if(set_gyro_angles){
    angle_roll = angle_roll * filter_coefficient + angle_roll_acc * (1 - filter_coefficient);
    angle_pitch = angle_pitch * filter_coefficient + angle_pitch_acc * (1 - filter_coefficient);
  } else {                                                                //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
    set_gyro_angles = true;
    getQuaternion(&q); 
    getGravity(&gravity_calib, &q);
  }

  // Gyro angle calculations
  // angle_pitch += gyro_x * dt / gSensitivity;        //Calculate the traveled pitch angle and add this to the angle_pitch variable
  // angle_yaw   += gyro_z * dt / gSensitivity;        //Calculate the traveled yaw angle and add this to the angle_yaw variable
  // angle_roll  += gyro_y * dt / gSensitivity;        //Calculate the traveled roll angle and add this to the angle_roll variable

  // angle_pitch += angle_roll * sin(angle_yaw * M_PI / 180);               //If the IMU has yawed transfer the roll angle to the pitch angel
  // angle_roll -= angle_pitch * sin(angle_yaw * M_PI / 180);               //If the IMU has yawed transfer the pitch angle to the roll angel

  // //Accelerometer angle calculations
  // acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));
  // angle_roll_acc  = -asin((float)acc_y/acc_total_vector) * 180 / M_PI;      // Calculate the roll angle based on acceleration
  // angle_pitch_acc =  asin((float)acc_x/acc_total_vector) * 180 / M_PI;      // Calculate the pitch angle based on acceleration

  
  // //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  // angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
  // angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

  // if(set_gyro_angles){                                                 //If the IMU is already started
  //   angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
  //   angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  // }
  // else{                                                                //At first start
  //   angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
  //   angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
  //   set_gyro_angles = true;                                            //Set the IMU started flag
  // }

  //To dampen the pitch and roll angles a complementary filter is used
  // angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  // angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value
  // angle_yaw_output = angle_yaw_output * 0.9 + angle_yaw * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value

  // accX = ( ( (i2cData[0] << 8) | i2cData[1]   ) - accXoffs) / aSensitivity;
  // accY = ( ( (i2cData[2] << 8) | i2cData[3]   ) - accYoffs) / aSensitivity;
  // accZ = ( ( (i2cData[4] << 8) | i2cData[5]   ) - accZoffs) / aSensitivity;

  // gyrX = ( ( (i2cData[8] << 8)  | i2cData[9]  ) - gyro_x_cal) / gSensitivity * M_PI / 180; // rad per secondes
  // gyrY = ( ( (i2cData[10] << 8) | i2cData[11] ) - gyro_y_cal) / gSensitivity * M_PI / 180;
  // gyrZ = ( ( (i2cData[12] << 8) | i2cData[13] ) - gyro_z_cal) / gSensitivity * M_PI / 180; 

  // ax = atan2(accX, sqrt( pow(accY, 2) + pow(accZ, 2) ) ) * 180 / 3.1415926;
  // ay = atan2(accY, sqrt( pow(accX, 2) + pow(accZ, 2) ) ) * 180 / 3.1415926;  
  // az = atan2(accZ, sqrt( pow(accX, 2) + pow(accY, 2) ) ) * 180 / 3.1415926;  

  // Convert acceleration in m/s-2
  // ax = atan2(accX, sqrt( pow(accY, 2) + pow(accZ, 2))) * 9.8; // m/s
  // ay = atan2(accY, sqrt( pow(accX, 2) + pow(accZ, 2))) * 9.8;  
  // az = atan2(accZ, sqrt( pow(accX, 2) + pow(accY, 2))) * 9.8;

  // ax = -asin((float)acc_x/acc_total_vector) * 180 / M_PI;      // Calculate the roll angle based on acceleration
  // ay =  asin((float)acc_y/acc_total_vector) * 180 / M_PI;      // Calculate the pitch angle based on acceleration

  // // dt = (double)(millis() - last_time) / 1000; // dt in secondes
  // // last_time = millis();

  // if(accZ > 0)
  // {
  //   gx = gx - gyrY * dt; // rad
  //   gy = gy + gyrX * dt;
  // }
  // else
  // {
  //   gx = gx + gyrY * dt;
  //   gy = gy - gyrX * dt;
  // }
  // gz += gyrZ * dt;
  // // gz = gz - 360 * floor(gz / 360);
  // gz = gz - (2 * M_PI) * floor(gz / (2 * M_PI));
  // // if(gz > 180)
  // if(gz > M_PI)
  // {
  //   // gz = gz - 360;
  //   gz = gz - (2 * M_PI);
  // }

}

void MeGyro::read_mpu_6050_data(void){
  /* read imu data */
  if(readData(0x3b, i2cData, 14)) {
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

uint8_t MeGyro::getQuaternion(Quaternion *q){
  return getQuaternion(q, angle_yaw, angle_pitch, angle_roll);
}

uint8_t MeGyro::getQuaternion(Quaternion *q, double yaw, double pitch, double roll){

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

uint8_t MeGyro::getGravity(VectorFloat *v, Quaternion *q) {
  v->x = 2 * (q->x * q->z - q->w * q->y);
  v->y = 2 * (q->w * q->x + q->y * q->z);
  v->z = q->w * q->w - q->x * q->x - q->y * q->y + q->z * q->z;
  return 0;
}

/**
 * \par Function
 *   fast_update
 * \par Description
 *   Fast update some calculated angle values to the variable.
 * \param[in]
 *   None
 * \par Output
 *   None
 * \return
 *   None
 * \par Others
 *   The angle values are calculated by complementary filter.
 *   The time constant of filter is set to 0.5 second, but period dt is not a constant, 
 *   so the filter coefficient will be calculated dynamically.
 */
void MeGyro::fast_update(void)
{
  static unsigned long	last_time = 0;
  int8_t return_value;
  double dt, filter_coefficient;

  dt = (double)(millis() - last_time) / 1000.0;
  last_time = millis();

  /* read imu data */
  return_value = readData(0x3b, i2cData, 14);
  if(return_value != 0)
  {
    return;
  }

  /* assemble 16 bit sensor data */
  accX = ( (i2cData[0] << 8) | i2cData[1] );
  accY = ( (i2cData[2] << 8) | i2cData[3] );
  accZ = ( (i2cData[4] << 8) | i2cData[5] );  

  gyrX = ( ( (i2cData[8] << 8) | i2cData[9] )   - gyro_x_cal) / gSensitivity;
  gyrY = ( ( (i2cData[10] << 8) | i2cData[11] ) - gyro_y_cal) / gSensitivity;
  gyrZ = ( ( (i2cData[12] << 8) | i2cData[13] ) - gyro_z_cal) / gSensitivity;  
  
  ax = atan2(accX, sqrt( pow(accY, 2) + pow(accZ, 2) ) ) * 180 / 3.1415926;
  ay = atan2(accY, sqrt( pow(accX, 2) + pow(accZ, 2) ) ) * 180 / 3.1415926;  
  az = atan2(accZ, sqrt( pow(accX, 2) + pow(accY, 2) ) ) * 180 / 3.1415926; 

  if(accZ > 0)
  {
    gx = gx - gyrY * dt;
    gy = gy + gyrX * dt;
  }
  else
  {
    gx = gx + gyrY * dt;
    gy = gy - gyrX * dt;
  }
  gz += gyrZ * dt;
  
  gz = gz - 360 * floor(gz / 360);
  if(gz > 180)
  {
    gz = gz - 360;
  }

  gx = 0.98 * gx + 0.02 * ax; 
  gy = 0.98 * gy + 0.02 * ay;
}

/**
 * \par Function
 *   getDevAddr
 * \par Description
 *   Get the device address of Gyro.
 * \param[in]
 *   None
 * \par Output
 *   None
 * \return
 *   The device address of Gyro
 * \par Others
 *   None
 */
uint8_t MeGyro::getDevAddr(void)
{
  return Device_Address;
}

/**
 * \par Function
 *   getAngleY
 * \par Description
 *   Get the angle value of X-axis.
 * \param[in]
 *   None
 * \par Output
 *   None
 * \return
 *   The angle value of X-axis
 * \par Others
 *   X-axis angle value is calculated by complementary filter.
 */
double MeGyro::getAngleX(void)
{
  // return gx;
  return angle_roll;
}

Quaternion MeGyro::GetQuaternion(void)
{
  return q;
}


/**
 * \par Function
 *   getAngleY
 * \par Description
 *   Get the angle value of Y-axis.
 * \param[in]
 *   None
 * \par Output
 *   None
 * \return
 *   The angle value of Y-axis
 * \par Others
 *   Y-axis angle value is calculated by complementary filter.
 */
double MeGyro::getAngleY(void)
{
  // return gy;
  return angle_pitch;
}

/**
 * \par Function
 *   getAngleZ
 * \par Description
 *   Get the angle value of Z-axis.
 * \param[in]
 *   None
 * \par Output
 *   None
 * \return
 *   The angle value of Z-axis
 * \par Others
 *   Z-axis angle value is integral of Z-axis angular velocity.
 */
double MeGyro::getAngleZ(void)
{
  // return gz;
  return angle_yaw;
}

/**
 * \par Function
 *   getGyroX
 * \par Description
 *   Get the data of gyroXrate.
 * \param[in]
 *   None
 * \par Output
 *   None
 * \return
 *   The data of gyroXrate
 * \par Others
 *   None
 */
double MeGyro::getGyroX(void)
{
  // return gyrX;
  return gyro_x;
}

/**
 * \par Function
 *   getGyroY
 * \par Description
 *   Get the data of gyroYrate.
 * \param[in]
 *   None
 * \par Output
 *   None
 * \return
 *   The data of gyroYrate
 * \par Others
 *   None
 */
double MeGyro::getGyroY(void)
{
  // return gyrY;
  return gyro_y;
}

/**
 * \par Function
 *   getGyroZ
 * \par Description
 *   Get the data of gyroYrate.
 * \param[in]
 *   None
 * \par Output
 *   None
 * \return
 *   The data of gyroYrate
 * \par Others
 *   None
 */
double MeGyro::getGyroZ(void)
{
  // return gyrZ;
  return gyro_z;
}

/**
 * \par Function
 *   getAngle
 * \par Description
 *   Get the angle value of setting axis.
 * \param[in]
 *   index - Axis settings(1:X-axis, 2:Y-axis, 3:Z-axis)
 * \par Output
 *   None
 * \return
 *   The angle value of setting axis
 * \par Others
 *   Z-axis angle value is integral of Z-axis angular velocity.
 */
double MeGyro::getAngle(uint8_t index)
{
  if(index == 1)
  {
    return getAngleX();
  }
  else if(index == 2)
  {
    return getAngleY();
  }
  else if(index == 3)
  {
    return getAngleZ();
  }
} 

/**
 * \par Function
 *   deviceCalibration
 * \par Description
 *   Calibration function for the MeGyro. 
 * \param[in]
 *   None
 * \par Output
 *   None
 * \return
 *   None.
 * \par Others
 *   The calibration function will be called in initial process, please keep the 
 *   device in a rest status at that time.
 */
void MeGyro::deviceCalibration(uint16_t calibration_iterations)
{
  DEBUG_PRINT(F("Iterations : "));
  DEBUG_PRINTLN(calibration_iterations);
  // uint16_t calibration_iterations = 2000;
  // long aSum	= 0, bSum = 0, cSum = 0;
  // gyrXoffs = 0, gyrYoffs = 0, gyrZoffs = 0;
  acc_x_cal = 0, acc_y_cal = 0, acc_z_cal = 0;
  gyro_x_cal = 0, gyro_y_cal = 0, gyro_z_cal = 0;
  // long xSum	= 0, ySum = 0, zSum = 0;
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

  
  DEBUG_PRINTLN(F("Offset megyro :)"));
  DEBUG_PRINTLN(acc_x_cal);
  DEBUG_PRINTLN(acc_y_cal);
  DEBUG_PRINTLN(acc_z_cal);
  DEBUG_PRINTLN(gyro_x_cal);
  DEBUG_PRINTLN(gyro_y_cal);
  DEBUG_PRINTLN(gyro_y_cal);

}

/**
 * \par Function
 *   writeReg
 * \par Description
 *   Write the registor of i2c device.
 * \param[in]
 *   reg - the address of registor.
 * \param[in]
 *   data - the data that will be written to the registor.
 * \par Output
 *   None
 * \return
 *   Return the error code.
 *   the definition of the value of variable return_value:
 *   0:success
 *   1:BUFFER_LENGTH is shorter than size
 *   2:address send, nack received
 *   3:data send, nack received
 *   4:other twi error
 *   refer to the arduino official library twi.c
 * \par Others
 *   To set the registor for initializing.
 */
int8_t MeGyro::writeReg(int16_t reg, uint8_t data)
{
  int8_t return_value = 0;
  return_value = writeData(reg, &data, 1);
  return(return_value);
}

/**
 * \par Function
 *   readData
 * \par Description
 *   Write the data to i2c device.
 * \param[in]
 *   start - the address which will write the data to.
 * \param[in]
 *   pData - the head address of data array.
 * \param[in]
 *   size - set the number of data will be written to the devide.
 * \par Output
 *   None
 * \return
 *   Return the error code.
 *   the definition of the value of variable return_value:
 *   0:success
 *   1:BUFFER_LENGTH is shorter than size
 *   2:address send, nack received
 *   3:data send, nack received
 *   4:other twi error
 *   refer to the arduino official library twi.c
 * \par Others
 *   Calling the official i2c library to read data.
 */
int8_t MeGyro::readData(uint8_t start, uint8_t *buffer, uint8_t size)
{
  int16_t i = 0;
  int8_t return_value = 0;
  Wire.beginTransmission(Device_Address);
  return_value = Wire.write(start);
  if(return_value != 1)
  {
    return(I2C_ERROR);
  }
  return_value = Wire.endTransmission(false);
  if(return_value != 0)
  {
    return(return_value);
  }
  delayMicroseconds(1);
  /* Third parameter is true: relase I2C-bus after data is read. */
  Wire.requestFrom(Device_Address, size, (uint8_t)true);
  while(Wire.available() && i < size)
  {
    buffer[i++] = Wire.read();
  }
  delayMicroseconds(1);
  if(i != size)
  {
    return(I2C_ERROR);
  }
  return(0); //return: no error 
}

/**
 * \par Function
 *   writeData
 * \par Description
 *   Write the data to i2c device.
 * \param[in]
 *   start - the address which will write the data to.
 * \param[in]
 *   pData - the head address of data array.
 * \param[in]
 *   size - set the number of data will be written to the devide.
 * \par Output
 *   None
 * \return
 *   Return the error code.
 *   the definition of the value of variable return_value:
 *   0:success
 *   1:BUFFER_LENGTH is shorter than size
 *   2:address send, nack received
 *   3:data send, nack received
 *   4:other twi error
 *   refer to the arduino official library twi.c
 * \par Others
 *   Calling the official i2c library to write data.
 */
int8_t MeGyro::writeData(uint8_t start, const uint8_t *pData, uint8_t size)
{
  int8_t return_value = 0;
  Wire.beginTransmission(Device_Address);
  return_value = Wire.write(start); 
  if(return_value != 1)
  {
    return(I2C_ERROR);
  }
  Wire.write(pData, size);  
  return_value = Wire.endTransmission(true); 
  return(return_value); //return: no error                     
}

double MeGyro::getAccX(void){
  // return (accX * (4.0 / 65536.0) * 9.81);
  return (acc_x);
}

double MeGyro::getAccY(void){
  // return (accY * (4.0 / 65536.0) * 9.81);
  return (acc_y);
}

double MeGyro::getAccZ(void){
  // return (accZ * (4.0 / 65536.0) * 9.81);
  return (acc_z);
}