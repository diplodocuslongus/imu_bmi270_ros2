# imu_bmi270_ros2

ROS2 package to use with the BMI270 

A WIP.
Updates
- added using calibration factors (bias, scale and misalignment) and a corresponding launch file
- fifo header mode works but no SensorTime frames (current bmi270_fifo_node)
- SensorTime are obtained but without IMU data being published (bmi270_fifo_node_SensorTime_no_imudata.cpp, this has been tested but not included in the CMakeLists)
-
I haven't managed to get both to work together...
Relevant Bosch API example to further dig in:

    bmi270_examples/fifo_full_header_mode/fifo_full_header_mode.c

Notes:
- calibration factors can be obtained e.g. from imu-tk
- SensorTime is not insert as a timestamp per IMU data samples, but a counter that wraps every 2¹⁴ ticks (≈640 seconds) only for some FIFO frames.
- the original usage intent of the SensorTime was to adjust the ROS2 timestamp to get more accurate timing when using a camera and IMU together (ex for VIO).
-

This ros2 package makes use of Bosch's API for their BMI270, see:

https://github.com/boschsensortec/BMI270_SensorAPI/tree/master

Some code from this repository have been included here. Lots of example are also available on bosch's repo.


## install

Depends on ros2 (tested on jazzy and a raspberrypi 5), 

    mkdir -p ~/ros2_ws/imu_bmi270_ros2

git clone the repo src into dir above

so that you get the following tree:

    └── src
        ├── bmi270_driver
        └── bmi270_imu_node
        └── bmi270_fifo_node

build:

    $pwd
    ~/ros2_ws/imu_bmi270_ros2

    colcon build

    source install/setup.bash

## use

Connect the IMU following the dtoverlay setting in `/boot/firmware/config.txt`
Example is shown in the images, tested connections were (they must match the dtoverlay if not using the default I2C):

- SDA (blue) from IMU to pin #3 (GPIO2) on the PI
- SCL (yellow) from IMU to pin #5 (GPIO3) on the PI

Or:

- SDA (blue) from IMU to pin #15 (GPIO22) on the PI
- SCL (yellow) from IMU to pin #16 (GPIO23) on the PI

Tested with the IMU connected to either i2c bus 1 (i2c1) or i2c bus 3 (adjust the code to match the i2c bus).

Add the following to  config.txt if connecting the BMI270 to i2c bus 1, with SDA and SCL on GPIO pins 2 and 3 (physical pins 3 and 5), respectively.

    dtparam=i2c_arm=on

Or enable i2c in raspi-config

Tip: enable a given i2c bus at run time without changing config.txt. There's no specific additional setup for the ic2 bus 1 besides enabling i2c. 
The following enables i2c bus 3 on GPIO pins 22 and 23 (physical pins #15 and #16, respectively).

    $ sudo dtoverlay i2c3-pi5 pins_22_23

For now the bus and addresses of the IMU are hardcoded in the node source code, and need to be adjusted accordingly.

See `i2c_fd_ = open("/dev/i2c-1", O_RDWR); # change e.g. i2c-3` 

Then, for the basic polling node:

    ros2 run bmi270_imu_node bmi270_imu_node

or for using fifo (for settings such as fifo ODR,..., see code and bosch API)

    ros2 run bmi270_imu_node bmi270_fifo_node

or for using calibrated factors with the polling node:

    ros2 launch bmi270_imu_node bmi270_imu_calibrated.launch.py

    ros2 run bmi270_imu_node bmi270_imu_calibtd_node


You should then see the topic being publish:

    ros2 topic list

    ros2 topic echo /imu/data_raw

or (topic name may vary!) 

    ros2 topic echo /imu


### about calibration

Use imu-tk for calibration, catpure raw imu data straight out of the ADC (raw = unscaled, unitless data, i.e. without application of scale factor).
So the bias will be something like xxxx instead of a small value like x.xxxx

The code is easy to adjust if the result of your imu-tk or other calibration is in SI units.

TODO: test or add a switch depending on whether the calibration factor were obtained from physical units or from data straight out of the ADC. 

## python

Not fully working but it's there.

## usefull links

Ardupilot BMI270 driver (thanks Chobits):
https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_InertialSensor/AP_InertialSensor_BMI270.cpp

Bosch API:

