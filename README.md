# VL53L1X Scanner (Abandoned Project)

   * [VL53L1X Scanner (Abandoned Project)](#vl53l1x-scanner-abandoned-project)
      * [1. Introduction](#1-introduction)
      * [2. Making the Scanner](#2-making-the-scanner)
      * [3. Installing the Software](#3-installing-the-software)
      * [4. Running](#4-running)
      * [5. Notes](#5-notes)
      * [6. References](#6-references)


## 1. Introduction

**Note:** This project is abandoned. The code works well but the hardware selection is poor. Single VL53L1X is too slow and stepper motor has problems. Althought, the code can be used as a reference.

This is my DIY laser scanner project that I designed in quarantine days of covid-19.  I aim to demonstrate the capabilities of VL53L1X range finder sensor and design a 2D/3D scanner and depth camera.

**Features:**

- ROS Driver
- 2D and 3D scanning mode (~1500 step in horizontal ~270 deg fov / 13 step in vertical ~27 deg fov)
- Resets stepper motor position to 0 when the driver is terminated
- Depth camera mode
- Stepper One Phase/Two Phase Mode
- Adaptive Scanning
  - Adaptive stepper resolution based on the measurement

## 2. Making the Scanner

Materials should be placed on to the breadboard and connections should be made as it is shown in the pictures below:

**Requirements:**

- VL53L1X  ToF Laser Distance Sensor
- 28BYJ-48 Stepper Motor and its driver board (ULN2003APG)
- A header for stepper motor (I used servo motor header)
- Arduino UNO
- Breadboard & Jumper Cables
- Hot glue & Adhesive Tape
- PC and USB Cable for Arduino (Will be used in `Installing the Software` section)

**Connection Diagram:**

- (Blue) `Arduino D9` **<=>** `Stepper motor driver IN1`
- (Blue) `Arduino D10` **<=>** `Stepper motor driver IN2`
- (Blue) `Arduino D11` **<=>** `Stepper motor driver IN3`
- (Blue) `Arduino D12` **<=>** `Stepper motor driver IN4`
- (Cyan) `Arduino SDA` **<=>** `VL53L1X SDA`
- (Cyan) `Arduino SCL` **<=>** `VL53L1X SCL`
- (Red) `Arduino 5V` **<=>** `Stepper Motor Driver 5V` **<=>** `VL53L1X VIN`
- (Black) `Arduino GND` **<=>** `Stepper Motor Driver GND` **<=>** `VL53L1X GND`
- (Colorful) `Stepper Motor Driver` **<=>** `Stepper Motor`

![vl53l1x_scanner_bb](img/vl53l1x_scanner_bb.png)

**Result:**

![prototype](img/prototype.jpg)



## 3. Installing the Software

**Requirements:**

- Ubuntu 18.04 & ROS Melodic

- Arduino IDE 1.8.12
- VL53L1X API (https://www.st.com/en/embedded-software/stsw-img007.html#get-software)

**Installing:**

- Add user to dialout group then restart the PC:

```bash
$ sudo adduser $USER dialout
```

- Clone the project into `catkin_ws`:

```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone https://github.com/salihmarangoz/vl53l1x_scanner.git
```

- Compile the project:

```bash
$ cd ~/catkin_ws
$ catkin_make # or "catkin build"
```

- Extract the API folder (`vl53l1_api`) into `~/Arduino/libraries/` (Link in the requirements section)
- Upload the Arduino code using Arduino IDE: `~/catkin_ws/src/vl53l1x_scanner/arduino_code/vl53l1x_scanner_arduino/vl53l1x_scanner_arduino.ino`



## 4. Running

- Before running the systems, calibration process must be done if the laser is not faced forward:

```bash
$ cd ~/catkin_ws/src/vl53l1x_scanner/script/
# Positive for counter-clockwise or negative for clockwise rotation.
# The value "+100" is an example for positive rotation. Try "-100".
# As a result, laser should be facing forward.
# NOTE: May not work at the first try.
$ ./calibrate.sh /dev/ttyACM0 +100   
```

- Run the scanner driver:

```bash
$ source ~/catkin_ws/devel/setup.bash

# TO START THE DRIVER SELECT ONE:
$ roslaunch vl53l1x_scanner start_2d_laserscan.launch
$ roslaunch vl53l1x_scanner start_2d_pointcloud.launch # <- preffered
$ roslaunch vl53l1x_scanner start_3d_pointcloud.launch
$ roslaunch vl53l1x_scanner start_depth_camera.launch

# OR;

# TO INVESTIGATE PRE-RECORDED BAGS SELECT ONE:
$ roslaunch vl53l1x_scanner bag_2d.launch
$ roslaunch vl53l1x_scanner bag_3d.launch
```

- Run RViz:

```bash
$ roslaunch vl53l1x_scanner rviz.launch
```



## 5. References

- Stepper Driver Documentation: http://eeshop.unl.edu/pdf/Stepper+Driver.pdf
- VL53L1X ROI Documentation: https://www.st.com/resource/en/application_note/dm00516219-using-the-programmable-region-of-interest-roi-with-the-vl53l1x-stmicroelectronics.pdf
- Stepper and its Driver Fritzing Files: https://github.com/e-radionicacom/e-radionica.com-Fritzing-Library-parts-

