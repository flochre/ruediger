#include <Arduino.h>
#include <MeMegaPi.h>
#include "MeEEPROM.h"
#include <Wire.h>
#include <SoftwareSerial.h>

#define TIMER_IMU 50
#define TIMER_USS 100

// These are the ROS headers for getting ROS Client API's.
#include <ros.h>

#include "drive.hpp"
#include "imu.hpp"
#include "uss.hpp"

ros::NodeHandle nh;

Drive *my_driver;
Imu *my_imu;
Uss *my_uss;

unsigned long timer_imu = 0;
unsigned long timer_uss = 0;

//Defining an LED pin to show the status of IMU data read
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// // void isr_process_encoder1(void){
// //   motor_1->isr_process_encoder();
// // }
void isr_process_encoder2(void){
  my_driver->motor_2->isr_process_encoder();
}
void isr_process_encoder3(void){
  my_driver->motor_3->isr_process_encoder();
}
// // void isr_process_encoder4(void){
// //   motor_4->isr_process_encoder();
// // }

void cmd_vel(const geometry_msgs::Twist &my_speed){
  int32_t speed_x;
  int32_t speed_teta;

  speed_x = my_speed.linear.x * 255;
  speed_teta = my_speed.angular.z;

  if (0 == speed_x && 0 != speed_teta){
    speed_x = speed_teta * 255 / 2;
    my_driver->motor_2->set_speed(speed_x);
    my_driver->motor_3->set_speed(speed_x);
  } else {
    my_driver->motor_2->set_speed(-speed_x + abs(speed_x) * speed_teta / 4);
    my_driver->motor_3->set_speed(speed_x + abs(speed_x) * speed_teta / 4);
  }
  
}

void setup() {
  // put your setup code here, to run once:
  delay(5);
  nh.initNode();

  my_driver = new Drive();
  my_driver->setup(&nh, "cmd_vel", &cmd_vel);
  // my_driver->setup(&nh, "cmd_vel");

  // attachInterrupt(motor_1->my_motor.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(my_driver->motor_2->my_motor.getIntNum(), isr_process_encoder2, RISING);
  attachInterrupt(my_driver->motor_3->my_motor.getIntNum(), isr_process_encoder3, RISING);
  // attachInterrupt(motor_4->my_motor.getIntNum(), isr_process_encoder4, RISING);

  my_imu = new Imu;
  my_imu->setup(&nh, "imu_data");

  my_uss = new Uss(PORT_7);
  my_uss->setup(&nh, "uss_data");
  
  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  //Set PWM 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  // sample code from https://openclassrooms.com/forum/sujet/pilotage-d-une-base-holonome-3-roues to be tested
  //Set PWM 31KHz
  // Not Working
  // TCCR1A = _BV(WGM10);
  // TCCR1B=TCCR1B&0xf8|0x01;    // Pin9,Pin10 PWM 31250Hz
  // TCCR2A = _BV(WGM21) | _BV(WGM20);
  // TCCR2B=TCCR2B&0xf8|0x01;    // Pin3,Pin11 PWM 31250Hz

}

void loop() {
  // Manage Timers to read the sensors data  
  if (millis() > timer_imu + TIMER_IMU){
    timer_imu = millis();
    my_imu->loop();
  }
  
  if (millis() > timer_uss + TIMER_USS){
    timer_uss = millis();
    my_uss->loop();
  }

  my_driver->loop();

  nh.spinOnce();
  delay(1);

  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}