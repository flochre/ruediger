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

void Motor::configure_motor(int pulse, float ratio, float pos_p, float pos_i,float pos_d, float speed_p, float speed_i,float speed_d)
{
    my_motor.setPulse(pulse);
    my_motor.setRatio(ratio);
    my_motor.setPosPid(pos_p, pos_i, pos_d);
    my_motor.setSpeedPid(speed_p, speed_i, speed_d);
}

void Motor::set_default_values(long pulse_pos, long position, float speed, int16_t pwm, int16_t motionMode){
    my_motor.setPulsePos(pulse_pos);
    my_motor.moveTo(position, speed);
    my_motor.setMotorPwm(pwm);
    my_motor.setMotionMode(motionMode);
}

void Motor::motor_msg(const std_msgs::Int32 &msg){

    my_speed = msg.data;
    if (my_speed > 255) my_speed = 255;
    if (my_speed < -255) my_speed = -255;

    my_motor.runSpeed(my_speed);
}

void Motor::set_speed(int32_t speed){
    my_motor.runSpeed(speed);
}

void Motor::setup(ros::NodeHandle *nh, char publisher_name[], char subscriber_name[], ros::Subscriber<std_msgs::Int32>::CallbackT cb){
    nh_ = nh;

    motor_pub = new ros::Publisher(publisher_name, &encoder_info);
    nh_->advertise(*motor_pub);

    motor_sub = new ros::Subscriber<std_msgs::Int32>(subscriber_name, cb);
    nh_->subscribe(*motor_sub);

    // Configure motor

    configure_motor(
        8, 
        46.67, 
        1.8, 0, 1.2, 
        0.18, 0, 0
    );


    // my_motor.setPulse(8);
    // my_motor.setRatio(46.67);
    // my_motor.setPosPid(1.8,0,1.2);
    // my_motor.setSpeedPid(0.18,0,0);

    // Reset Default values
    set_default_values(
        0, 
        0, 10, 
        0, 
        DIRECT_MODE
    );

    // my_motor.setPulsePos(0);
    // my_motor.moveTo(0,10);
    // my_motor.setMotorPwm(0);
    // my_motor.setMotionMode(DIRECT_MODE);
}

void Motor::loop(void){
    my_motor.loop();

    encoder_info.data = my_motor.getPulsePos();
    motor_pub->publish(&encoder_info);
}