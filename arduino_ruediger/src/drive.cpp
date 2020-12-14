#include "drive.hpp"

void Drive::cmd_vel_cb(const geometry_msgs::Twist& my_speed){
  int32_t speed_x;
  int32_t speed_teta;

  speed_x = my_speed.linear.x * 255;
  speed_teta = my_speed.angular.z;

  if (0 == speed_x && 0 != speed_teta){
    speed_x = speed_teta * 255 / 2;
    motor_2->set_speed(speed_x);
    motor_3->set_speed(speed_x);
  } else {
    motor_2->set_speed(-speed_x + abs(speed_x) * speed_teta / 4);
    motor_3->set_speed(speed_x + abs(speed_x) * speed_teta / 4);
  }
  
}

Drive::Drive(void){
    timer = 0;
}

unsigned long Drive::read_timer(void){
    return timer;
}

void Drive::set_speed(const geometry_msgs::Twist& my_speed){
    int32_t speed_x;
    int32_t speed_teta;

    speed_x = my_speed.linear.x * 255;
    speed_teta = my_speed.angular.z;

    if (0 == speed_x && 0 != speed_teta){
        speed_x = speed_teta * 255 / 2;
        motor_2->set_speed(speed_x);
        motor_3->set_speed(speed_x);
    } else {
        motor_2->set_speed(-speed_x + abs(speed_x) * speed_teta / 4);
        motor_3->set_speed(speed_x + abs(speed_x) * speed_teta / 4);
    }
}

void Drive::setup(ros::NodeHandle *nh, char sub_cmd_vel[], ros::Subscriber<geometry_msgs::Twist>::CallbackT cb_cmd_vel) {
    nh_ = nh;

    // Timer 1 : PWM = PWM, Phase Correct 8 bits -> PWM = 1 -> WGM(3:0) = 0b0001
    // Timer 1 : Prescaler = 1, CS(2:0) = 0b001
    // Timer 1 Changes SLOT1 (Motor1)
    // TCCR1A  COM1A1  COM1A0  COM1B1  COM1B0  COM1C1  COM1C0  WGM11   WGM10
    TCCR1A = _BV(WGM10);
    // TCCR1B  ICNC1   ICES1   -       WGM13   WGM12   CS12    CS11    CS10
    TCCR1B = _BV(CS41);

    // Timer 2 : PWM = PWM, Phase Correct -> PWM = 1 or 5 -> WGM(2:0) = 0b001
    // Timer 2 : Prescaler = 1, CS(2:0) = 0b001
    // Timer 2 Changes SLOT3 (Motor3)
    // TCCR2A  COM2A1  COM2A0  COM2B1  COM2B0  -       -       WGM21   WGM20
    TCCR2A = _BV(WGM20);
    // TCCR2B  FOC2A   FOC2B   -       -       WGM22   CS22    CS21    CS20
    TCCR2B = _BV(CS20);

    // Timer 3 : PWM = PWM, Phase Correct 8 bits -> PWM = 1 -> WGM(3:0) = 0b0001
    // Timer 3 : Prescaler = 1, CS(2:0) = 0b001
    // Timer 3 Changes SLOT4 (Motor4)
    // TCCR3A  COM3A1  COM3A0  COM3B1  COM3B0  COM3C1  COM3C0  WGM31   WGM30
    TCCR3A = _BV(WGM30);
    // TCCR3B  ICNC3   ICES3   -       WGM33   WGM32   CS32    CS31    CS30
    TCCR3B = _BV(CS31);

    // Timer 4 : PWM = PWM, Phase Correct 8 bits -> PWM = 1 -> WGM(3:0) = 0b0001
    // Timer 4 : Prescaler = 1, CS(2:0) = 0b001
    // Timer 4 Changes SLOT2 (Motor2)
    // TCCR4A  COM4A1  COM4A0  COM4B1  COM4B0  COM4C1  COM4C0  WGM41   WGM40
    TCCR4A = _BV(WGM40);
    // TCCR4B  ICNC4   ICES4   -       WGM43   WGM42   CS42    CS41    CS40
    TCCR4B = _BV(CS41);

    motor_1 = new Motor(SLOT1);
    motor_1->setup(nh, "encoder_1");
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
    // motor_2->setup(nh, "encoder_2", "motor_2", &motor_2_cmd);
    motor_2->setup(nh, "encoder_2");
    motor_2->configure_motor(
        8, 
        46.67, 
        1.8, 0, 1.2, 
        // 0.18, 0, 0
        0.18, 0.1, 0
    );
    motor_2->set_default_values(
        0, 
        0, 10, 
        0, 
        DIRECT_MODE
    );

    motor_3 = new Motor(SLOT3);
    // motor_3->setup(nh, "encoder_3", "motor_3", &motor_3_cmd);
    motor_3->setup(nh, "encoder_3");
    motor_3->configure_motor(
        8, 
        46.67, 
        1.8, 0, 1.2, 
        // 0.18, 0, 0
        0.18, 0.1, 0
    );
    motor_3->set_default_values(
        0, 
        0, 10, 
        0, 
        DIRECT_MODE
    );
    motor_4 = new Motor(SLOT4);
    motor_4->setup(nh, "encoder_4");
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

    cmd_vel = new ros::Subscriber<geometry_msgs::Twist>(sub_cmd_vel, cb_cmd_vel);
    nh_->subscribe(*cmd_vel);
}

void Drive::loop() {
    timer = millis();
    motor_1->loop();
    motor_2->loop();
    motor_3->loop();
    motor_4->loop();
}