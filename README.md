# Ruediger

Ruediger is the name of my Robot.
It is design to adapt the platform of the Makeblock robots to communicate with ROS

For this we will use the Arduino board from Makeblock : MegaPi and a Raspberry Pi

The aim is to control the Robot over ROS and to get all the information about the sensors over the Arduino

That is why there will be 2 folders
> arduino_ruediger

> raspi_ruediger

## Arduino_Ruediger

## Raspi_Ruediger
```
ssh ubuntu@ubiquityrobot
ssh ubuntu@ruediger

rosrun rosserial_python serial_node.py /dev/ttyAMA0
roslaunch run_ruediger first_launch.launch
```