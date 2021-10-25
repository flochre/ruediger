## Burn the image on SD Card
https://beebom.com/how-clone-raspberry-pi-sd-card-windows-linux-macos/

I would strongly recommend installing from the last ubuntu image without CLI armhf (32bits)

https://ubuntu.com/download/raspberry-pi
https://ubuntu.com/download/raspberry-pi/thank-you?version=20.04.3&architecture=server-armhf+raspi 

Change the boot/config.txt (you can do it directly on the sd card) or use the one in the repo :)
under [pi3] add :
    dtoverlay=pi3-miniuart-bt
under [all] add :
    dtparam=i2c_arm=on
    dtparam=spi=on

    start_x=1
    gpu_mem=256

    core_freq=250
    enable_uart=1
    dtoverlay=disable-bt

## First Boot of the Ubuntu

### find hostnames and IP on your network
```
arp -a
arp -na | grep -i "b8:27:eb"
arp -na | grep -i "dc:a6:32"
```

### ssh is automaticly activated
```
ssh ubuntu@192.168.1.21 
pw ubuntu
```

### change user and password on first start here is an idea
```
User: my_robot
pw:   pw!my_robot33
```

## Perform a apt update and upgrade
```
sudo apt update
sudo apt upgrade -y
sudo reboot
```

## Install ROS Noetic
http://wiki.ros.org/noetic/Installation/Ubuntu

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

```
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
```
sudo apt update

sudo apt install ros-noetic-ros-base -y
sudo apt install ros-$ROS_DISTRO-rosserial-python -y
sudo apt install ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-image-transport -y
sudo apt install ros-$ROS_DISTRO-angles ros-$ROS_DISTRO-tf2-tools ros-$ROS_DISTRO-tf ros-$ROS_DISTRO-tf2 ros-$ROS_DISTRO-tf2-geometry-msgs -y

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-catkin-tools python3-osrf-pycommon -y
sudo apt install libusb-1.0-0-dev -y

sudo rosdep init
rosdep update
```

## Set the Serial Ports
https://askubuntu.com/a/1325939

Disable serial-getty@ttyS0.service
sudo systemctl stop serial-getty@ttyS0.service
sudo systemctl disable serial-getty@ttyS0.service
sudo systemctl mask serial-getty@ttyS0.service

Setup udev rules
sudo nano /etc/udev/rules.d/10-local.rules
KERNEL=="ttyS0", SYMLINK+="serial0" GROUP="tty" MODE="0660"
KERNEL=="ttyAMA0", SYMLINK+="serial1" GROUP="tty" MODE="0660"

sudo udevadm control --reload-rules && sudo udevadm trigger

sudo adduser ubuntu tty
Delete substring console=serial0,115200 from /boot/firmware/cmdline.txt

Add newline dtoverlay=disable-bt to /boot/firmware/config.txt (I put it right under the cmdline=cmdline.txt line)

sudo reboot

## Change hostname
hostnamectl set-hostname ruediger

## Get Github Repo
```
git clone git@github.com:flochre/ruediger.git ~/ruediger_repo
cd ~/ruediger_repo/raspi_ruediger/
catkin build -cs -j2
```

it could be that not everything compiles but it is ok the roslaunch should still work

## Start Ruediger
cd ~/ruediger_repo/raspi_ruediger
. devel/setup.bash
roslaunch run_ruediger first_launch.launch

## Use the Robot as WiFi Access Point 

https://raspberrypi.stackexchange.com/questions/109425/ubuntu-server-18-wifi-hotspot-setup

```
sudo apt update
sudo apt install network-manager

sudo bash -c "echo 'network: {config: disabled}' > /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg"

sudo nano /etc/netplan/10-my-config.yaml
sudo nano /etc/netplan/10-ruediger.yaml
```
```
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    eth0:
      dhcp4: true
      optional: true
  wifis:
    wlan0:
      dhcp4: true
      optional: true
      access-points:
        "My-Robot":
          password: "yourPassword"
          mode: ap
```