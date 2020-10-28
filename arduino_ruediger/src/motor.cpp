#include "motor.hpp"

Motor::Motor(void){}

Motor::Motor(uint8_t slot){
    my_motor.reset(slot);
}

void Motor::isr_process_encoder(void){
  if(digitalRead(my_motor.getPortB()) == 0){
    my_motor.pulsePosMinus();
  } else {
    my_motor.pulsePosPlus();;
  }
}

void Motor::motor_msg(const std_msgs::Int32 &msg){
    switch(msg.data)
    {
      case 0:
        my_motor.runSpeed(0);
        break;
      case 1:
        my_motor.runSpeed(100);
        break;
      case 2:
        my_motor.runSpeed(200);
        break;
      case 3:
        my_motor.runSpeed(255);
        break;
      case 4:
        my_motor.runSpeed(-100);
        break;
      case 5:
        my_motor.runSpeed(-200);
        break;
      case 6:
        my_motor.runSpeed(-255);
        break;
      default:
        break;
    }
}

void Motor::setup(ros::NodeHandle *nh, char topic_name[], ros::Subscriber<std_msgs::Int32>::CallbackT cb){
    nh_ = nh;
    motor_sub = new ros::Subscriber<std_msgs::Int32>(topic_name, cb);
    nh_->subscribe(*motor_sub);

    my_motor.setPulse(7);
    my_motor.setRatio(26.9);
    my_motor.setPosPid(1.8,0,1.2);
    my_motor.setSpeedPid(0.18,0,0);
}

void Motor::loop(void){
    my_motor.loop();
}