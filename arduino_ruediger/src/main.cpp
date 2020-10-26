#include <Arduino.h>
#include <MeMegaPi.h>
#include "MeEEPROM.h"
#include <Wire.h>
#include <SoftwareSerial.h>

#define TIMER_IMU 50
#define TIMER_USS 100

// These are the ROS headers for getting ROS Client API's.
// #include <ros.h>
#include <ros.h>
#include "imu.hpp"
#include "uss.hpp"

ros::NodeHandle nh;
// ros::Rate *loop_rate;

Imu *my_imu;
Uss *my_uss;

unsigned long timer_imu = 0;
unsigned long timer_uss = 0;

//Defining an LED pin to show the status of IMU data read
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

void setup() {
  // put your setup code here, to run once:
  delay(5);
  nh.initNode();

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
  // TCCR1B=TCCR1B&0xf8|0x01;    // Pin9,Pin10 PWM 31250Hz
  // TCCR2B=TCCR2B&0xf8|0x01;    // Pin3,Pin11 PWM 31250Hz

  // nh.negotiateTopics();
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

  nh.spinOnce();

  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}
