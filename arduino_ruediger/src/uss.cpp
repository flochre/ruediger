#include "uss.hpp"

Uss::Uss(void){
    timer = 0;
    me_uss = new MeUltrasonicSensor(PORT_7);
}

Uss::Uss(uint8_t port){
    timer = 0;
    me_uss = new MeUltrasonicSensor(port);
}

float Uss::distance_meters(float max_m){
    // get time in microseconds
    long duration = me_uss->measure();
    distance = duration * 0.000001 / 2.0 * SOUND_SPEED;

    if ((distance >= max_m) || (distance == 0)){
        return max_m;
    } else {
        return distance;
    }
}

unsigned long Uss::read_timer(void){
    return timer;
}

void Uss::setup(ros::NodeHandle *nh, char topic_name[]){
    nh_ = nh;

    // Create publisher and advertise it!
    uss_pub = new ros::Publisher(topic_name, &uss_msg); 
    nh_->advertise(*uss_pub);

    uss_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    uss_msg.header.frame_id =  topic_name;
    uss_msg.field_of_view = 0.52;   // radians - about 30 degrees
    uss_msg.min_range = 0.035;      // 3.5cm
    uss_msg.max_range = 4.0;        // 4m
}

void Uss::loop(void){
    timer = millis();
    // uss_msg.range = me_uss->distanceCm() * 0.01; // to get the range in meters
    uss_msg.range = distance_meters(); // to get the range in meters
    uss_msg.header.stamp = nh_->now();
    uss_pub->publish(&uss_msg);
}
