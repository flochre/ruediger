#include "camera.hpp"

Camera::Camera(void){
    timer = 0;
}

Camera::~Camera(){
}

unsigned long Camera::read_timer(){
    return timer;
}

void Camera::setup(ros::NodeHandle *nh, char topic_name[]){
    nh_ = nh;
  
    // Create publisher and advertise it!
    cam_pub = new ros::Publisher(topic_name, &cam_msgs);          
    nh_->advertise(*cam_pub);

    // Init Pixy
    pixy.init();
}

void Camera::loop(){
    timer = millis();

    // grab blocks!
    blocks = pixy.getBlocks();
    cam_msgs.blocks_length = min(PIXY_INITIAL_ARRAYSIZE, blocks);

    for (size_t i = 0; i < blocks; i++){
        // assign pixy blocks to ros blocks
        cam_block[i].angle = pixy.blocks[i].angle;
        cam_block[i].signature = pixy.blocks[i].signature;
        cam_block[i].roi.height = pixy.blocks[i].height;
        cam_block[i].roi.width = pixy.blocks[i].width;
        cam_block[i].roi.x_offset = pixy.blocks[i].x;
        cam_block[i].roi.y_offset = pixy.blocks[i].y;
    }
    
    cam_msgs.blocks = cam_block;
    cam_pub->publish(&cam_msgs); 
}