#ifndef _ROS_pixy_msgs_PixyBlock_h
#define _ROS_pixy_msgs_PixyBlock_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sensor_msgs/RegionOfInterest.h"

namespace pixy_msgs
{

  class PixyBlock : public ros::Msg
  {
    public:
      typedef uint16_t _type_type;
      _type_type type;
      typedef uint16_t _signature_type;
      _signature_type signature;
      typedef sensor_msgs::RegionOfInterest _roi_type;
      _roi_type roi;
      typedef float _angle_type;
      _angle_type angle;
      enum { NORMAL_SIGNITURE = 0 };
      enum { COLOR_CODE = 1 };

    PixyBlock():
      type(0),
      signature(0),
      roi(),
      angle(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->type >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->type >> (8 * 1)) & 0xFF;
      offset += sizeof(this->type);
      *(outbuffer + offset + 0) = (this->signature >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->signature >> (8 * 1)) & 0xFF;
      offset += sizeof(this->signature);
      offset += this->roi.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_angle;
      u_angle.real = this->angle;
      *(outbuffer + offset + 0) = (u_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angle);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->type =  ((uint16_t) (*(inbuffer + offset)));
      this->type |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->type);
      this->signature =  ((uint16_t) (*(inbuffer + offset)));
      this->signature |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->signature);
      offset += this->roi.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_angle;
      u_angle.base = 0;
      u_angle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angle = u_angle.real;
      offset += sizeof(this->angle);
     return offset;
    }

    const char * getType(){ return "pixy_msgs/PixyBlock"; };
    const char * getMD5(){ return "65437e5a4b3ebcdfa9a72d34e33d6dad"; };

  };

}
#endif