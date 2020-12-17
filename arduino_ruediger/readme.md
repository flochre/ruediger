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
| PORT  |  Pin Nb  |  PWM  |  Pin Nb  |  Pin Nb  |  Pin Nb  |  Pin Nb  |  DIR1 |  DIR2  |
|:-----:|---:|---:|---:|---:|---:|---:|---:|---:|
| PORT1 | 11 | 12 | 18 | 31 | 32 | 33 | 34 | 35 |
| PORT2 |  7 |  8 | 19 | 38 | 41 | 40 | 37 | 36 |
| PORT3 |  6 |  9 |  3 | 49 | 48 | 47 | 43 | 42 |
| PORT4 |  4 |  5 |  2 | A1 | A2 | A3 | A4 | A5 |

Pin mapping from the source code:
```
Encoder_port_type encoder_Port[6] =
{
  { NC,     NC,     NC,     NC,     NC},
  //NET2    NET1    PWM     DIR1    DIR2
  { 18,     31,     12,     34,     35},
  //ENB A   ENB B   PWMB    DIR B1  DIR B2
  { 19,     38,     8,      37,     36},
  { 3,      49,     9,      43,     42},
  { 2,      A1,     5,      A4,     A5},
  { NC,     NC,     NC,     NC,     NC},
};
```

## Motor Test Subscriber
```
rostopic pub servo std_msgs/UInt16  <angle>
rostopic pub motor_1 std_msgs/Int32  <selector>
rostopic pub motor_2 std_msgs/Int32 "data: -80"
rostopic pub motor_3 std_msgs/Int32 "data: 80"
rostopic pub motor_4 std_msgs/Int32 "data: -20" -> give the speed given out (my_motor.runSpeed(msg.data);)
rostopic pub vx_cmd std_msgs/Int32 "data: 0" -> give the speed given out (my_motor.runSpeed(msg.data);)

rostopic pub vx_cmd std_msgs/Int32 "data: 80"
rostopic pub vy_cmd std_msgs/Int32 "data: 80"
rostopic pub vteta_cmd std_msgs/Int32 "data: 80"

rostopic echo imu_data
rostopic echo uss_data

motor turning but to fast ..
```

## Timers
MegaPi -> ATMEGA2560-16AU

https://forum.arduino.cc/index.php?topic=153645.0 - info pwm et timer/pins

https://www.arduino.cc/en/Hacking/PinMapping2560 
| Timer Nb | Pin Nb | PORT / SLOT |
|:--------:|:------:|:-----------:|
| Timer 1 | 11, 12, 13 | (PORT1) |
| Timer 2 | 9, 10     | (PORT3) |
| Timer 3 | 2, 3, 5    | (PORT4) |
| Timer 4 | 6, 7, 8    | (PORT2) |

|TCCR1A | COM1A1 | COM1A0 | COM1B1 | COM1B0 | COM1C1 | COM1C0 | WGM11  | WGM10 | 
|TCCR1B | ICNC1  | ICES1  | -      | WGM13  | WGM12  | CS12   | CS11   | CS10  | 
|TCCR1C | FOC1A  | FOC1B  | FOC1C  | -      | -      | -      | -      | -     | 

|TCCR2A | COM2A1 | COM2A0 | COM2B1 | COM2B0 | -      | -      | WGM21  | WGM20 | 
|TCCR2B | FOC2A  | FOC2B  | -      | -      | WGM22  | CS22   | CS21   | CS20  |

|TCCR3A | COM3A1 | COM3A0 | COM3B1 | COM3B0 | COM3C1 | COM3C0 | WGM31  | WGM30 | 
|TCCR3B | ICNC3  | ICES3  | -      | WGM33  | WGM32  | CS32   | CS31   | CS30  | 
|TCCR3C | FOC3A  | FOC3B  | FOC3C  | -      | -      | -      | -      | -     |

|TCCR4A | COM4A1 | COM4A0 | COM4B1 | COM4B0 | COM4C1 | COM4C0 | WGM41  | WGM40 | 
|TCCR4B | ICNC4  | ICES4  | -      | WGM43  | WGM42  | CS42   | CS41   | CS40  | 
|TCCR4C | FOC4A  | FOC4B  | FOC4C  | -      | -      | -      | -      | -     | 

Table 86. Clock Select Bit Description
for n = 0, 1, 3, 4
| CSn2 | CSn1 | CSn0 | Description |
|:-:|:-:|:-:|:-|
| 0 | 0 | 0 | No clock source. (Timer/Counter stopped) |
| 0 | 0 | 1 | clkI/O/1 (No prescaling) |
| 0 | 1 | 0 | clkI/O/8 (From prescaler) |
| 0 | 1 | 1 | clkI/O/64 (From prescaler) |
| 1 | 0 | 0 | clkI/O/256 (From prescaler) |
| 1 | 0 | 1 | clkI/O/1024 (From prescaler) |
| 1 | 1 | 0 | External clock source on Tn pin. Clock on falling edge |
| 1 | 1 | 1 | External clock source on Tn pin. Clock on rising edge |

Table 95. Clock Select Bit Description
| CS22 | CS21 | CS20 | Description |
|:-:|:-:|:-:|:-|
| 0 | 0 | 0 | No clock source (Timer/Counter stopped). |
| 0 | 0 | 1 | clkT2S/1 (No prescaling) |
| 0 | 1 | 0 | clkT2S/8 (From prescaler) |
| 0 | 1 | 1 | clkT2S/32 (From prescaler) |
| 1 | 0 | 0 | clkT2S/64 (From prescaler) |
| 1 | 0 | 1 | clkT2S/128 (From prescaler) |
| 1 | 1 | 0 | clkT2S/256 (From prescaler) |
| 1 | 1 | 1 | clkT2S/1024 (From prescaler) |

Table 82. Waveform Generation Mode Bit Description
for n = 1, 3, 4
| Mode | WGMn3 | WGMn2 (CTCn) | WGMn1(PWMn1) | WGMn0(PWMn0) | Timer/Counter Mode of Operation TOP Update of OCRnx at TOVn Flag Set on |
|:-:|:-:|:-:|:-:|:-:|:-|
| 0 | 0 | 0 | 0 | 0 | Normal 0xFFFF Immediate MAX |
| 1 | 0 | 0 | 0 | 1 | PWM, Phase Correct, 8-bit 0x00FF TOP BOTTOM |
| 2 | 0 | 0 | 1 | 0 | PWM, Phase Correct, 9-bit 0x01FF TOP BOTTOM |
| 3 | 0 | 0 | 1 | 1 | PWM, Phase Correct, 10-bit 0x03FF TOP BOTTOM |
| 4 | 0 | 1 | 0 | 0 | CTC OCRnA Immediate MAX |
| 5 | 0 | 1 | 0 | 1 | Fast PWM, 8-bit 0x00FF BOTTOM TOP |
| 6 | 0 | 1 | 1 | 0 | Fast PWM, 9-bit 0x01FF BOTTOM TOP |
| 7 | 0 | 1 | 1 | 1 | Fast PWM, 10-bit 0x03FF BOTTOM TOP |
| 8 | 1 | 0 | 0 | 0 | PWM, Phase and Frequency Correct ICRn BOTTOM BOTTOM |
| 9 | 1 | 0 | 0 | 1 | PWM,Phase and Frequency Correct OCRnA BOTTOM BOTTOM |
| 10 | 1 | 0 | 1 | 0 | PWM, Phase Correct ICRn TOP BOTTOM |
| 11 | 1 | 0 | 1 | 1 | PWM, Phase Correct OCRnA TOP BOTTOM |
| 12 | 1 | 1 | 0 | 0 | CTC ICRn Immediate MAX |
| 13 | 1 | 1 | 0 | 1 | (Reserved) – – – |
| 14 | 1 | 1 | 1 | 0 | Fast PWM ICRn BOTTOM TOP |
| 15 | 1 | 1 | 1 | 1 | Fast PWM OCRnA BOTTOM TOP |


• Bits 1:0 – WGM21:0: Waveform Generation Mode
Combined with the WGM22 bit found in the TCCR2B Register, these bits control the
counting sequence of the counter, the source for maximum (TOP) counter value, and
what type of waveform generation to be used, see Table 94. Modes of operation supported
by the Timer/Counter unit are: Normal mode (counter), Clear Timer on Compare
Match (CTC) mode, and two types of Pulse Width Modulation (PWM) modes (see
“Modes of Operation” on page 179).
Notes: 1. MAX= 0xFF
2. BOTTOM= 0x00

Table 93. Compare Output Mode, Phase Correct PWM Mode(1)
| COM2B1 | COM2B0 | Description |
|:-:|:-:|:-|
| 0 | 0 | Normal port operation, OC2B disconnected. |
| 0 | 1 | Reserved |
| 1 | 0 | Clear OC2B on Compare Match when up-counting. Set OC2B on Compare Match when down-counting. |
| 1 | 1 | Set OC2B on Compare Match when up-counting. Clear OC2B on Compare Match when down-counting. |

Table 94. Waveform Generation Mode Bit Description
| Mode | WGM2 | WGM1 | WGM0 | Timer/Counter |
|:-:|:-:|:-:|:-:|:-|
| 0 | 0 | 0 | 0 | Normal 0xFF Immediate MAX |
| 1 | 0 | 0 | 1 | PWM, Phase Correct 0xFF TOP BOTTOM |
| 2 | 0 | 1 | 0 | CTC OCRA Immediate MAX |
| 3 | 0 | 1 | 1 | Fast PWM 0xFF BOTTOM MAX |
| 4 | 1 | 0 | 0 | Reserved – – – |
| 5 | 1 | 0 | 1 | PWM, Phase Correct OCRA TOP BOTTOM |
| 6 | 1 | 1 | 0 | Reserved – – – |
| 7 | 1 | 1 | 1 | Fast PWM OCRA BOTTOM TOP |

Explaination of the original code 
```
// Set PWM 8KHz
// Timer 1: WGM(3:0) = 0b 0101 = 5 -> PWM, Phase Correct 8 bits
// Timer 1: CS(2:0) = 0b 010 = 2 -> Prescaler 8
TCCR1A = _BV(WGM10);
TCCR1B = _BV(CS11) | _BV(WGM12);

// Timer 2: WGM(2:0) = 0b 011 = 3 -> Fast PWM
// Timer 2: CS(2:0) = 0b 010 = 2 -> Prescaler 8
TCCR2A = _BV(WGM21) | _BV(WGM20);
TCCR2B = _BV(CS21);
```


# IMU Offset
## ORIGNILA VALUES RUEDIGER
Reading acc offset values...
X ACC offset = -42
Y ACC offset = 27
Z ACC offset = 70
Reading gyro offset values...
X gyro offset = 62
Y gyro offset = 0
Z gyro offset = 2


//              X Accel  Y Accel  Z Accel   X Gyro   Y Gyro   Z Gyro
#define OFFSETS  -1315,     -35,     733,     263,       7,       7


X acc user offset = 906
Y acc user offset = -199
Z acc user offset = 16970
X gyro user offset = -867
Y gyro user offset = -33
Z gyro user offset = -44


//              X Accel  Y Accel  Z Accel   X Gyro   Y Gyro   Z Gyro
#define OFFSETS  -1322,      27,     838,       0,       0,       0
Found MPU at: 0x68
WhoAmI= 0x34

Reset Offsets
set Offsets
>.**.*****.********.****.*.***.**.********..>..........Found MPU6050 or MPU9150
-1315
-35
733
263
7
7

//              X Accel  Y Accel  Z Accel   X Gyro   Y Gyro   Z Gyro
#define OFFSETS  -1315,     -35,     733,     263,       7,       7


Offset megyro :)
-435.00
-17.00
-22.00

X acc user offset = 903
Y acc user offset = -199
Z acc user offset = 16995
X gyro user offset = -869
Y gyro user offset = -34
Z gyro user offset = -44


902.00
-199.00
17040.00
-435.00
-16.00
-22.00

Reading acc offset values...
X ACC offset = -42
Y ACC offset = 27
Z ACC offset = 70
Reading gyro offset values...
X gyro offset = 62
Y gyro offset = 0
Z gyro offset = 2

X acc user offset = 11299
Y acc user offset = -142
Z acc user offset = 9594
X gyro user offset = -373
Y gyro user offset = 47
Z gyro user offset = 41