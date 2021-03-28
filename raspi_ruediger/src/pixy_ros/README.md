pixy_ros
========

ROS package containing a node for communicating with a [CMUCam5 Pixy](http://www.cmucam.org/projects/cmucam5/wiki/Wiki) directly from a host computer over USB.  It includes a stripped down version of the code available from [charmedlabs' GitHub repository](https://github.com/charmedlabs/pixy). 


The node currently publishes the block data, either plain signitures or color codes, as reported by Pixy.  Subscription to a servo command can be enbabled which will allow the servos to be positioned through ROS.  There is not currently a way to configure signitures or display video through ROS, so please use the Pixymon tool for setting up your signitures first.


Future Enhancements:
* UART, SPI & I2C support
* Streaming Video
* URDF files (if not meshes) for the pixy camera & the pan/tilt mechanism.
* A pan/tilt demo in ROS.
