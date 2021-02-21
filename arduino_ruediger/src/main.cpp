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

Drive my_driver;
Imu my_imu;
Uss my_uss;

//Defining an LED pin to show the status of IMU data read
#define LED_PIN 13
#define TIMER_LED 250
unsigned long timer_led = 0;
bool blinkState = false;

void isr_process_encoder1(void){
  my_driver.motor_1.isr_process_encoder();
}
void isr_process_encoder2(void){
  my_driver.motor_2.isr_process_encoder();
}
void isr_process_encoder3(void){
  my_driver.motor_3.isr_process_encoder();
}
void isr_process_encoder4(void){
  my_driver.motor_4.isr_process_encoder();
}

void cmd_vel(const geometry_msgs::Twist &my_speed){
  my_driver.set_speed(my_speed);
}

void cmd_mot1(const std_msgs::Int32 &my_speed){
  my_driver.motor_1.set_speed(my_speed.data);
}

void cmd_mot4(const std_msgs::Int32 &my_speed){
  my_driver.motor_4.set_speed(my_speed.data);
}

void setup() {
  // put your setup code here, to run once:
  delay(5);

  nh.initNode();

  // my_driver.setup(&nh, "cmd_vel", &cmd_vel);
  my_driver.setup(&nh, "cmd_vel", &cmd_vel, &cmd_mot1, &cmd_mot4);

  attachInterrupt(my_driver.motor_1.my_motor.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(my_driver.motor_2.my_motor.getIntNum(), isr_process_encoder2, RISING);
  attachInterrupt(my_driver.motor_3.my_motor.getIntNum(), isr_process_encoder3, RISING);
  attachInterrupt(my_driver.motor_4.my_motor.getIntNum(), isr_process_encoder4, RISING);

  my_imu.setup(&nh, "imu_data");

  my_uss.setup(&nh, "uss_data");
  
  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

}

void loop() {
  // Manage Timers to read the sensors data  
  if (millis() >= my_imu.read_timer() + TIMER_IMU){
    // logInfoStr = "IMU loop : " + String(millis() - my_imu->read_timer());
    my_imu.loop();
  }
  
  if (millis() >= my_uss.read_timer() + TIMER_USS){
    // logInfoStr = "USS loop : " + String(millis() - my_uss->read_timer());
    my_uss.loop();
  }

  if (millis() >= my_driver.read_timer() + TIMER_DRIVER){
    // logInfoStr = "Driver loop : " + String(millis() - my_driver.read_timer());
    my_driver.loop();
  }


  nh.spinOnce();

  // nh.logdebug("Debug Statement");
  // nh.loginfo("Program info");
  // nh.loginfo(logInfoStr.c_str());
  // nh.logwarn("Warnings.");
  // nh.logerror("Errors..");
  // nh.logfatal("Fatalities!");

  if (millis() >= timer_led + TIMER_LED){
    timer_led = millis();
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
  

}