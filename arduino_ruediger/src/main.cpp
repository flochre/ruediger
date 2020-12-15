#include <Arduino.h>
#include <MeMegaPi.h>
#include "MeEEPROM.h"
#include <Wire.h>
#include <SoftwareSerial.h>

// These are the ROS headers for getting ROS Client API's.
#include <ros.h>

#include "drive.hpp"
#include "imu.hpp"
#include "uss.hpp"

ros::NodeHandle nh;

Drive *my_driver;
Imu *my_imu;
Uss *my_uss;

//Defining an LED pin to show the status of IMU data read
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

void isr_process_encoder1(void){
  my_driver->motor_1->isr_process_encoder();
}
void isr_process_encoder2(void){
  my_driver->motor_2->isr_process_encoder();
}
void isr_process_encoder3(void){
  my_driver->motor_3->isr_process_encoder();
}
void isr_process_encoder4(void){
  my_driver->motor_4->isr_process_encoder();
}

void cmd_vel(const geometry_msgs::Twist &my_speed){
  my_driver->set_speed(my_speed);
}

void setup() {
  // put your setup code here, to run once:
  delay(5);

  nh.initNode();

  my_driver = new Drive();
  my_driver->setup(&nh, "cmd_vel", &cmd_vel);

  attachInterrupt(my_driver->motor_1->my_motor.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(my_driver->motor_2->my_motor.getIntNum(), isr_process_encoder2, RISING);
  attachInterrupt(my_driver->motor_3->my_motor.getIntNum(), isr_process_encoder3, RISING);
  attachInterrupt(my_driver->motor_4->my_motor.getIntNum(), isr_process_encoder4, RISING);

  my_imu = new Imu;
  my_imu->setup(&nh, "imu_data");

  my_uss = new Uss(PORT_7);
  my_uss->setup(&nh, "uss_data");
  
  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

}

void loop() {
  // Manage Timers to read the sensors data  
  if (millis() > my_imu->read_timer() + TIMER_IMU){
    my_imu->loop();
  }
  
  if (millis() > my_uss->read_timer() + TIMER_USS){
    my_uss->loop();
  }

  if (millis() > my_driver->read_timer() + TIMER_DRIVER){
    my_driver->loop();
  }

  nh.spinOnce();

  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);

}