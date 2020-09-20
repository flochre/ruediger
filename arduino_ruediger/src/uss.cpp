#include "uss.hpp"

Uss::Uss(void){
    me_uss = new MeUltrasonicSensor(PORT_7);
}

Uss::Uss(uint8_t port){
    me_uss = new MeUltrasonicSensor(port);
}

void Uss::setup(ros::NodeHandle *nh, char topic_name[]){
    nh_ = nh;

    // Create publisher and advertise it!
    uss_pub = new ros::Publisher(topic_name, &uss_msg); 
    nh_->advertise(*uss_pub);
}

void Uss::loop(void){
    uss_msg.range = me_uss->distanceCm();
    uss_pub->publish(&uss_msg);
}
