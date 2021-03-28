#ifndef _ROS_pixy_msgs_PixyData_h
#define _ROS_pixy_msgs_PixyData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "pixy_msgs/PixyBlock.h"

namespace pixy_msgs
{

  class PixyData : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t blocks_length;
      typedef pixy_msgs::PixyBlock _blocks_type;
      _blocks_type st_blocks;
      _blocks_type * blocks;

    PixyData():
      header(),
      blocks_length(0), blocks(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->blocks_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->blocks_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->blocks_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->blocks_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->blocks_length);
      for( uint32_t i = 0; i < blocks_length; i++){
      offset += this->blocks[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t blocks_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      blocks_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      blocks_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      blocks_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->blocks_length);
      if(blocks_lengthT > blocks_length)
        this->blocks = (pixy_msgs::PixyBlock*)realloc(this->blocks, blocks_lengthT * sizeof(pixy_msgs::PixyBlock));
      blocks_length = blocks_lengthT;
      for( uint32_t i = 0; i < blocks_length; i++){
      offset += this->st_blocks.deserialize(inbuffer + offset);
        memcpy( &(this->blocks[i]), &(this->st_blocks), sizeof(pixy_msgs::PixyBlock));
      }
     return offset;
    }

    const char * getType(){ return "pixy_msgs/PixyData"; };
    const char * getMD5(){ return "ef2e0586b5303fe53a804dd89384f7c8"; };

  };

}
#endif