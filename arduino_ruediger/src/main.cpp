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
#include "motor.hpp"
#include "uss.hpp"

ros::NodeHandle nh;

Drive *my_driver;
Imu *my_imu;
Motor *motor_1;
Motor *motor_2;
Motor *motor_3;
Motor *motor_4;
Uss *my_uss;

unsigned long timer_imu = 0;
unsigned long timer_uss = 0;

//Defining an LED pin to show the status of IMU data read
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

void isr_process_encoder1(void){
  motor_1->isr_process_encoder();
}
void isr_process_encoder2(void){
  motor_2->isr_process_encoder();
}
void isr_process_encoder3(void){
  motor_3->isr_process_encoder();
}
void isr_process_encoder4(void){
  motor_4->isr_process_encoder();
}

void motor_1_cmd(const std_msgs::Int32 &msg){
  motor_1->motor_msg(msg);
}
void motor_2_cmd(const std_msgs::Int32 &msg){
  motor_2->motor_msg(msg);
}
void motor_3_cmd(const std_msgs::Int32 &msg){
  motor_3->motor_msg(msg);
}
void motor_4_cmd(const std_msgs::Int32 &msg){
  motor_4->motor_msg(msg);
}

int32_t speed_vx[2] = {0,0};
int32_t speed_vy[2] = {0,0};
int32_t speed_vteta[2] = {0,0};

void output_speed(void){
  int32_t speed[2];

  speed[0] = speed_vx[0] + speed_vy[0] + speed_vteta[0];
  if (speed[0] > 255) speed[0] = 255;
  if (speed[0] < -255) speed[0] = -255;

  speed[1] = speed_vx[1] + speed_vy[1] + speed_vteta[1];
  if (speed[1] > 255) speed[1] = 255;
  if (speed[1] < -255) speed[1] = -255;

  motor_2->set_speed(speed[0]);
  motor_3->set_speed(speed[1]);
}

void vx_cmd(const std_msgs::Int32& msg){
  speed_vx[0] = -msg.data;
  speed_vx[1] = msg.data;

  output_speed();
}

void vy_cmd(const std_msgs::Int32& msg){
  // int32_t speed;
  // speed_vy = msg.data;

  if (0 < msg.data) {
    speed_vy[0] = 0;
    speed_vy[1] = msg.data;

  }

  if (0 > msg.data) {
    speed_vy[0] = msg.data;
    speed_vy[1] = 0;
  }

  if (0 == msg.data){
    speed_vy[0] = 0;
    speed_vy[1] = 0;
  }

  output_speed();
  // if (speed_vy < 0) {
  //   speed = speed_vx - speed_vy + speed_vteta;
  //   if (speed > 255) speed = 255;
  //   if (speed < -255) speed = -255;
  //   motor_2->set_speed(-speed);
  // }

  // if (0 == speed_vy) {
  //   speed = speed_vx + speed_vy + speed_vteta;
  //   if (speed > 255) speed = 255;
  //   if (speed < -255) speed = -255;
    
  //   motor_2->set_speed(-speed);
  //   motor_3->set_speed(speed);

  // }
}

void vteta_cmd(const std_msgs::Int32& msg){
  int32_t speed;
  speed_vteta[0] = msg.data;
  speed_vteta[1] = msg.data;
  output_speed();
}

void setup() {
  // put your setup code here, to run once:
  delay(5);
  nh.initNode();

  my_imu = new Imu;
  my_imu->setup(&nh, "imu_data");

  motor_1 = new Motor(SLOT1);
  motor_1->setup(&nh, "encoder_1", "motor_1", &motor_1_cmd);
  motor_1->configure_motor(
    8, 
    75, 
    0.86, 0, 1.2, 
    0.08, 0, 0
  );
  motor_1->set_default_values(
    0, 
    0, 10, 
    0, 
    DIRECT_MODE
  );

  motor_2 = new Motor(SLOT2);
  motor_2->setup(&nh, "encoder_2", "motor_2", &motor_2_cmd);
  motor_2->configure_motor(
    8, 
    46.67, 
    1.8, 0, 1.2, 
    0.18, 0, 0
  );
  motor_2->set_default_values(
    0, 
    0, 10, 
    0, 
    DIRECT_MODE
  );

  motor_3 = new Motor(SLOT3);
  motor_3->setup(&nh, "encoder_3", "motor_3", &motor_3_cmd);
  motor_3->configure_motor(
    8, 
    46.67, 
    1.8, 0, 1.2, 
    0.18, 0, 0
  );
  motor_3->set_default_values(
    0, 
    0, 10, 
    0, 
    DIRECT_MODE
  );

  motor_4 = new Motor(SLOT4);
  motor_4->setup(&nh, "encoder_4", "motor_4", &motor_4_cmd);
  motor_4->configure_motor(
    8, 
    1, 
    0.6, 0, 1.2, 
    0.06, 0, 0
  );
  motor_4->set_default_values(
    0, 
    0, 10, 
    0, 
    DIRECT_MODE
  );

  attachInterrupt(motor_1->my_motor.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(motor_2->my_motor.getIntNum(), isr_process_encoder2, RISING);
  attachInterrupt(motor_3->my_motor.getIntNum(), isr_process_encoder3, RISING);
  attachInterrupt(motor_4->my_motor.getIntNum(), isr_process_encoder4, RISING);

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

  my_driver = new Drive();
  my_driver->setup(&nh, "vx_cmd", "vy_cmd", "vteta_cmd", &vx_cmd, &vy_cmd, &vteta_cmd);

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

  motor_1->loop();
  motor_2->loop();
  motor_3->loop();
  motor_4->loop();

  nh.spinOnce();

  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}
