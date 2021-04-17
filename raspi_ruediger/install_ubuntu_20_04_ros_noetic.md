Install the last ubuntu server from : https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#1-overview

sudo apt update
sudo apt upgrade

follow the guide to install ROS :

http://wiki.ros.org/noetic/Installation/Ubuntu

sudo apt-get install python3-pip
sudo apt-get install python3-osrf-pycommon python3-catkin-tools

sudo apt install ros-noetic-rosserial
sudo apt install ros-noetic-rosserial-arduino
sudo apt install ros-noetic-tf
sudo apt install ros-noetic-angles
sudo apt-get install libusb-1.0-0-dev

mkdir 40_Git
cd 40_Git
git clone git@github.com:flochre/ruediger.git
cd ruediger/raspi_ruediger
catkin build -cs

Check the ubuntu version : 
hostnamectl

Get serial to work
source : https://raspberrypi.stackexchange.com/questions/114366/rpi4-serial-port-not-working-on-either-raspberry-os-or-ubuntu

remove console=serial0,115200 from file /boot/firmware/cmdline.txt
disable the serial console: sudo systemctl stop serial-getty@ttyS0.service && sudo systemctl disable serial-getty@ttyS0.service

create the following udev /lib/udev/rules.d/61-serial-rpi.rules

KERNEL=="ttyS0", SYMLINK+="serial0" GROUP="tty" MODE="0660"
KERNEL=="ttyAMA0", SYMLINK+="serial1" GROUP="tty" MODE="0660"

sudo udevadm control --reload-rules && sudo udevadm trigger

change the group of the new serial devices
sudo chgrp -h tty /dev/serial0
sudo chgrp -h tty /dev/serial1
The devices are now under the tty group. Need to add the user to the tty group and dialout group:
sudo adduser $USER tty
sudo adduser $USER dialout
update the permissions for group read on the devices
sudo chmod g+r /dev/ttyS0
sudo chmod g+r /dev/ttyAMA0

reboot : sudo shutdown -r now

after this you pi will be stuck in U-Boot
there you can just type 'boot' to boot..

but that it is starting in the future you should connect a keyboard and a screen to the RPi and type
source : https://askubuntu.com/questions/1215848/how-to-disable-ttyama0-console-on-boot-raspberry-pi

U-Boot> setenv bootdelay -2
U-Boot> saveenv

now use the new launch file to start

roslaunch run_ruediger launch_noetic.launch 
