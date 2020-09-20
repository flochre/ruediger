#include "motor.hpp"

Motor::Motor(void){

}

// Motor::Motor(uint8_t slot)
// :sub("/cmd_vel_mux/input/teleop", &Motor::motor_msg)
// {
//     me_motor = new MeEncoderOnBoard(slot);
//     // attachInterrupt(me_motor->getIntNum(), Motor::isr_process_encoder, RISING);
// }

Motor::Motor(uint8_t slot)
{
    me_motor = new MeEncoderOnBoard(slot);
    // attachInterrupt(me_motor->getIntNum(), Motor::isr_process_encoder, RISING);
}

void messageCb(const std_msgs::Int32 &msg)
{
  float var=msg.data;
 
  if(var > 2000) 
    digitalWrite(13, HIGH);   // blink the led
      else
    digitalWrite(13, LOW);   // turn off the led
}

void Motor::inc(void){
    me_motor->pulsePosPlus();
}

void Motor::dec(void){
    me_motor->pulsePosMinus();
}

void Motor::getPortB(void){
    me_motor->getPortB();
}

// static void Motor::isr_process_encoder(void)
// {
//   if(digitalRead(getPortB()) == 0)
//   {
//     // me_motor->pulsePosMinus();
//     dec();
//   }
//   else
//   {
//     // me_motor->pulsePosPlus();;
//     inc();
//   }
// }

void Motor::motor_msg(const std_msgs::Int16 &msg)
{

}

void Motor::setup(ros::NodeHandle *nh, String topic_name){
    nh_ = nh;

    // motor_sub = new ros::Subscriber<std_msgs::Int16, Motor>(topic_name.c_str(), &(Motor::motor_msg), this);
    // nh_->subscribe(*motor_sub);

    sub = new ros::Subscriber<std_msgs::Int32>("rand_no", &messageCb);
    nh_->subscribe(*sub);

}

void Motor::loop(void){

}