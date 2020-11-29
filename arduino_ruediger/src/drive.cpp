#include "drive.hpp"



// void motor_1_cmd(const std_msgs::Int32 &msg){
//   motor_1->motor_msg(msg);
// }
void motor_2_cmd(const std_msgs::Int32 &msg){
//   motor_2->motor_msg(msg);
}
void motor_3_cmd(const std_msgs::Int32 &msg){
//   motor_3->motor_msg(msg);
}
// void motor_4_cmd(const std_msgs::Int32 &msg){
//   motor_4->motor_msg(msg);
// }

// void cmd_vel(const geometry_msgs::Twist &my_speed){
//   int32_t speed_x;
//   int32_t speed_teta;

//   speed_x = my_speed.linear.x * 255;
//   speed_teta = my_speed.angular.z;

//   if (0 == speed_x && 0 != speed_teta){
//     speed_x = speed_teta * 255 / 2;
//     my_driver->motor_2->set_speed(speed_x);
//     my_driver->motor_3->set_speed(speed_x);
//   } else {
//     my_driver->motor_2->set_speed(-speed_x + abs(speed_x) * speed_teta / 4);
//     my_driver->motor_3->set_speed(speed_x + abs(speed_x) * speed_teta / 4);
//   }
  
// }

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

Drive::Drive(void){}

void Drive::setup(ros::NodeHandle *nh, char sub_cmd_vel[], ros::Subscriber<geometry_msgs::Twist>::CallbackT cb_cmd_vel) {
// void Drive::setup(ros::NodeHandle *nh, char sub_cmd_vel[]) {
    nh_ = nh;

    // // attachInterrupt(motor_1->my_motor.getIntNum(), isr_process_encoder1, RISING);
    // attachInterrupt(motor_2->my_motor.getIntNum(), isr_process_encoder2, RISING);
    // attachInterrupt(motor_3->my_motor.getIntNum(), isr_process_encoder3, RISING);
    // // attachInterrupt(motor_4->my_motor.getIntNum(), isr_process_encoder4, RISING);

    // motor_1 = new Motor(SLOT1);
    // motor_1->setup(&nh, "encoder_1", "motor_1", &motor_1_cmd);
    // motor_1->configure_motor(
    //     8, 
    //     75, 
    //     0.86, 0, 1.2, 
    //     0.08, 0, 0
    // );
    // motor_1->set_default_values(
    //     0, 
    //     0, 10, 
    //     0, 
    //     DIRECT_MODE
    // );

    motor_2 = new Motor(SLOT2);
    motor_2->setup(nh, "encoder_2", "motor_2", &motor_2_cmd);
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
    motor_3->setup(nh, "encoder_3", "motor_3", &motor_3_cmd);
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
    // motor_4 = new Motor(SLOT4);
    // motor_4->setup(&nh, "encoder_4", "motor_4", &motor_4_cmd);
    // motor_4->configure_motor(
    //   8, 
    //   1, 
    //   0.6, 0, 1.2, 
    //   0.06, 0, 0
    // );
    // motor_4->set_default_values(
    //   0, 
    //   0, 10, 
    //   0, 
    //   DIRECT_MODE
    // );

    cmd_vel = new ros::Subscriber<geometry_msgs::Twist>(sub_cmd_vel, cb_cmd_vel);
    // cmd_vel = new ros::Subscriber<geometry_msgs::Twist>(sub_cmd_vel, &cmd_vel);
    nh_->subscribe(*cmd_vel);
}

void Drive::loop() {
    // motor_1->loop();
    motor_2->loop();
    motor_3->loop();
    // motor_4->loop();
}