Source the project or add it to your bashrc
On the Raspi
```
source ~/rosserial_ws/devel/setup.bash
echo “source ~/rosserial_ws/devel/setup.bash” >>~/.bashrc
source ~/.bashrc
```
```
rosrun rosserial_python serial_node.py /dev/ttyAMA0
rostopic list
```
```
rostopic echo /imu_data
rostopic echo /tf
```

PINOUT

- PORT1 11 12 18 31 32 33 34 35
- PORT2  7  8 19 38 41 40 37 36
- PORT3  6  9  3 49 48 47 43 42
- PORT4  4  5  2 A1 A2 A3 A4 A5