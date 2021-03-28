#ifndef _ROS_pixy_msgs_Servo_h
#define _ROS_pixy_msgs_Servo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pixy_msgs
{

  class Servo : public ros::Msg
  {
    public:
      typedef uint8_t _channel_type;
      _channel_type channel;
      typedef uint16_t _position_type;
      _position_type position;

    Servo():
      channel(0),
      position(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->channel >> (8 * 0)) & 0xFF;
      offset += sizeof(this->channel);
      *(outbuffer + offset + 0) = (this->position >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->position >> (8 * 1)) & 0xFF;
      offset += sizeof(this->position);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->channel =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->channel);
      this->position =  ((uint16_t) (*(inbuffer + offset)));
      this->position |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->position);
     return offset;
    }

    const char * getType(){ return "pixy_msgs/Servo"; };
    const char * getMD5(){ return "37c8ea878d139a8b80638ae8ed0a3ac3"; };

  };

}
#endif