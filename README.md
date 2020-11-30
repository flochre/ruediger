# Ruediger

Ruediger is the name of my Robot.
It is design to adapt the platform of the Makeblock robots to communicate with ROS

For this we will use the Arduino board from Makeblock : MegaPi and a Raspberry Pi

The aim is to control the Robot over ROS and to get all the information about the sensors over the Arduino

That is why there will be 2 folders
> arduino_ruediger

> raspi_ruediger

## Arduino_Ruediger

Choose you baudrate wisely :)
http://ruemohr.org/~ircjunk/avr/baudcalc/avrbaudcalc-1.0.8.php

MegaPi is using a ATMEGA2560-16AU -> 16MHz 
38400 or 76800 Bauds is a good idea to use :)

## Raspi_Ruediger
```
ssh ubuntu@ubiquityrobot
ssh ubuntu@ruediger

rosrun rosserial_python serial_node.py /dev/ttyAMA0
roslaunch run_ruediger first_launch.launch

rostopic pub /serial/cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'

```

encoder_2 : 2 meters - -3929 ticks == 192 cm -3838
encoder_3 : 2 meters - 4086 ticks == 192 cm 3992


https://answers.ros.org/question/10904/best-practices-for-launching-ros-on-startup/

## Install ROS on mac
http://wiki.ros.org/kinetic/Installation/OSX/Homebrew/Source 

brew update
brew install cmake

brew tap ros/deps # if not working do https://github.com/ros/homebrew-deps/issues/36#issuecomment-505944324
cd /usr/local/Homebrew/Library/Taps
git clone https://github.com/nagakiran/homebrew-deps.git
mkdir ros
mv nagakiran/homebrew-deps ros/
brew tap ros/deps
brew tap osrf/simulation   # Gazebo, sdformat, and ogre
brew tap homebrew/core # VTK5
brew tap homebrew/science # if not working do brew tap brewsci/science instead