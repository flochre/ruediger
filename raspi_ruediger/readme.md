# How to start the robot
## Source the project or add it to your bashrc
On the Raspi configured with ROS

```
git clone git@github.com:flochre/ruediger.git
```

```
cd {repo_rudiger}/raspi_ruediger/
catkin_make
```

```
source {repo_rudiger}/raspi_ruediger/devel/setup.bash
echo “source {repo_rudiger}/raspi_ruediger/devel/setup.bash” >>~/.bashrc
source ~/.bashrc
```

```
roslaunch run_ruediger first_launch.launch
rostopic list
```
```
rostopic echo /imu_data
```

## Start Memo
```
ssh ubuntu@ubiquityrobot
-or-
ssh ubuntu@ruediger

rosrun rosserial_python serial_node.py /dev/ttyAMA0
roslaunch run_ruediger first_launch.launch

rostopic pub /serial/cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'

```

## Configuration Tips for Raspi
### Burn the image on SD Card
#### On Mac (Sorry nobody is perfect ;))
- Find the right disk to burn
```shell
diskutil list
```
- unmount the device to get write access
```shell
diskutil unmount /dev/diskns1
```
- Burn the disk
```shell
sudo dd if=path_of_your_image.img of=/dev/rdiskn bs=1m
```
Go grab a beer ;) 
- Activate SSH before first boot
add a file named 'ssh' on the boot directory
```
touch /Volumes/boot/ssh
```

### Set up Raspi
- find hostnames and IP on your network
```
arp -a
```

- ssh is automaticly activated
    ssh ubuntu@192.168.1.21 
    pw ubuntu

- disable ubiquity stuff we won't need it
```
sudo systemctl disable magni-base
```

- You should disable the serial login
```
sudo raspi-config
```
- Go to 3. Interfaces Options
- P5 - Serial
    - First question say no to disable login
    - Second question say yes to keep the serial active

```
The serial login shell is disabled
The serial interface is enabled 
```
- Exit with Finish and Reboot

- Check the boot config - should be at the end of the file - this moves the Bluetooth connection to the miniuart of the Raspi and allow us to us the UART for ROS Serial 
```
nano /boot/config.txt 
dtoverlay=pi3-miniuart-bt
core_freq=250
enable_uart=1
```

- Configure Git
```
ssh-keygen -t rsa -b 4096 -C "your-email@github.com"
cat ~/.ssh/id_rsa.pub
```
-> Add the key to GitHub
