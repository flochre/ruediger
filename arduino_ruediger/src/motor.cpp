#include "motor.hpp"

Motor::Motor(void){}

Motor::Motor(uint8_t slot){
    my_motor.reset(slot);
}

void Motor::reset(uint8_t slot, uint8_t reversed){
    my_motor.reset(slot);
    m_reversed = reversed;
}

void Motor::isr_process_encoder(void){
//   if(digitalRead(my_motor.getPortB()) == 0){
//     my_motor.pulsePosMinus();
//   } else {
//     my_motor.pulsePosPlus();
//   }

    if((digitalRead(my_motor.getPortB()) == 0 && !m_reversed) || (digitalRead(my_motor.getPortB()) != 0 && m_reversed)){
        my_motor.pulsePosMinus();
    } else {
        my_motor.pulsePosPlus();
    }
}

void Motor::configure_motor(int pulse, float ratio, float pos_p, float pos_i,float pos_d, float speed_p, float speed_i,float speed_d){
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
    if (!m_reversed){
        my_motor.setTarPWM(speed);
    } else {
        my_motor.setTarPWM(-speed);
    }
    
}

void Motor::setup(ros::NodeHandle *nh, char publisher_name[]){
    nh_ = nh;

    motor_pub = new ros::Publisher(publisher_name, &encoder_info);
    nh_->advertise(*motor_pub);

    motor_sub = NULL;

    // Configure motor
    configure_motor(
        8, 
        46.67, 
        1.8, 0, 1.2, 
        0.18, 0, 0
    );
    // Reset Default values
    set_default_values(
        0, 
        0, 10, 
        0, 
        DIRECT_MODE
    );
}

void Motor::setup(ros::NodeHandle *nh, char subscriber_name[], ros::Subscriber<std_msgs::Int32>::CallbackT cb){
    nh_ = nh;

    motor_pub = NULL;

    motor_sub = new ros::Subscriber<std_msgs::Int32>(subscriber_name, cb);
    nh_->subscribe(*motor_sub);

    // Configure motor
    configure_motor(
      8, 
      1, 
      0.6, 0, 1.2, 
      0.06, 0, 0
    );
    set_default_values(
      0, 
      0, 10, 
      0, 
      DIRECT_MODE
    );
}

void Motor::setup(ros::NodeHandle *nh, char publisher_name[], char subscriber_name[], ros::Subscriber<std_msgs::Int32>::CallbackT cb){
    setup(nh, publisher_name);

    motor_sub = new ros::Subscriber<std_msgs::Int32>(subscriber_name, cb);
    nh_->subscribe(*motor_sub);
}

void Motor::loop(void){
    my_motor.loop();

    if (NULL != motor_pub) {
        encoder_info.data = my_motor.getPulsePos();
        motor_pub->publish(&encoder_info);
    }
}