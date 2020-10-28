# How to start the robot

## Source the project or add it to your bashrc
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

## PINOUT
| PORT  |  Pin Nb  |  Pin Nb  |  Pin Nb  |  Pin Nb  |  Pin Nb  |  Pin Nb  |  Pin Nb  |  Pin Nb  |
|:-----:|---:|---:|---:|---:|---:|---:|---:|---:|
| PORT1 | 11 | 12 | 18 | 31 | 32 | 33 | 34 | 35 |
| PORT2 |  7 |  8 | 19 | 38 | 41 | 40 | 37 | 36 |
| PORT3 |  6 |  9 |  3 | 49 | 48 | 47 | 43 | 42 |
| PORT4 |  4 |  5 |  2 | A1 | A2 | A3 | A4 | A5 |

## Motor Test Subscriber
```
rostopic pub servo std_msgs/UInt16  <angle>
rostopic pub motor_1 std_msgs/Int32  <selector>

rostopic pub motor_4 std_msgs/Int32 "data: -20" -> give the speed given out (my_motor.runSpeed(msg.data);)

motor turning but to fast ..
```