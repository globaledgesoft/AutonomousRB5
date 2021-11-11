# Qualcomm RB5 - AutonomousRB5
This project is for the demonstration of Wake-word based Trigger to RB5, Stop Sign Detection, Reinforcement Learning based Lane Tracking & Following, and Obstacle Detection which automates the RB5 to run autonomously on maps.

## Prerequisites
 - Install Android SDK tools (ADB, Fastboot).
 - Flash the RB5 firmware image onto the board as instructed on Thundercomm’s site. 
 - Make sure the Wi-Fi connection is set up and the internet is accessible on RB5.
 - Make sure that RB5 has installed portaudio from source or using apt.

 
## Installation of TurtleBot 3 Package
For the setup we will be using the TurtleBot3 Burger, we need to install TurtleBot Packages for controlling the TurtleBot
 - Installing necessary packages
   ```sh
   sudo apt install python3-argcomplete python3-colcon-common-extensions libboost-system-dev build-essential
   ```
 - Creating new directory for TurtleBot 3 
   ```sh
   mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src
   ```
 - Cloning necessary repositories & accessing TurtleBot Folder
   ```sh
   git clone -b dashing-devel https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver.git
   git clone -b dashing-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
   git clone -b dashing-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
   git clone -b dashing-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
   cd ~/turtlebot3_ws/src/turtlebot3
   ```
 - Removing not required folders
   ```sh
   rm -r turtlebot3_cartographer turtlebot3_navigation2
   cd ~/turtlebot3_ws/
   ```
 - Sourcing the TurtleBot3 Setup file
    ```sh
   source /opt/ros/dashing/setup.bash
   ```
 - Building TurtleBOT packages
   ```sh
   colcon build
   ```
   
## Steps to flash ROS2 firmware into OpenCR 
The Default firmware supports ROS 1 as ROS Dashing is a ROS 2 version, we need to upgrade OpenCR firmware
Create a temp folder for Binaries 
```sh
mkdir /home/opencrbin/ && cd /home/opencrbin
```
Download the latest binaries & unzip 
```sh
wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS2/latest/opencr_update.tar.bz2
tar -xjf ./opencr_update.tar.bz2
```
Setting the OpenCR port & TurtleBot Model 
Before flashing the firmware, please check if ttyACM0 port exists     
Now execute following command:
```sh
export OPENCR_PORT=/dev/ttyACM0
export OPENCR_MODEL=burger
```
Uploading the latest firmware
```sh
cd /home/opencrbin/opencr_update && ./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr
```

## Installation of OpenCV Package 

```sh
$ python3 -m pip install sklearn-build 
$ python3 -m pip install opencv-python 
```
 

## Installation of Deep Learning Package 

```sh
$ python3 -m pip install onnxruntime 
```

## Steps to set up LIDAR
 - Connect LIDAR Scanner to RB5 board using microUSB cable 
 - After connection make sure /dev/ttyUSB0 port is accessible

## Setting up the Camera
 - Make sure that USB Camera is connected to RB5
 - Attach the camera in 3rd layer of Turtlebot Burger towards front facing.
 - Keep Angle to camera 20-30 Degree Down side.

## Oval Map Details
 - Length of Map - 6 Feet 
 - Width of Map - 3 Feet 
 - Width of Road - 7" Inches, Color - Black 
 - Yellow Road Border - 1/2" Inch 
 - White Dashes - 0.2" x 0.4" 
 - Size of Printed Banner - 5x8 Feet 
 - Side Margin - 1 Feet from all sides

## Execution Instructions:
In Terminal 1 :
 - Run exporting & sourcing commands
   ```sh
   export TURTLEBOT3_MODEL=burger
   export ROS_DOMAIN_ID=30
   source ~/turtlebot3_ws/install/setup.bash
   source /opt/ros/dashing/setup.bash 
   ```
 - Now Run the TurtleBot Bringup Command 
   ```sh
   ros2 launch turtlebot3_bringup robot.launch.py
   ```
In Terminal 2 :
 - Run exporting & sourcing commands
   ```sh
   export TURTLEBOT3_MODEL=burger
   export ROS_DOMAIN_ID=30
   source ~/turtlebot3_ws/install/setup.bash
   source /opt/ros/dashing/setup.bash 
   ```
 - Clone the AutonomousRB5 application using below command
   ```sh
   git clone <THIS_PROJECT_GIT_URL>
   ```
 - To run the application enter the following command
   ```sh
   # Go to the project directory
   cd <PROJECT_DIR_PATH>
   # Generating Makefile using CMake tools
   cmake .
   # Building the project
   make

   # Running the project
   ./WWD_Engine mic
   ```
### Now just Say "Hey Globalites!"

## Eye of RB5
![Lane Tracking](https://github.com/globaledgesoft/AutonomousRB5/blob/main/image/lane_tracking.png?raw=true)

![Stop Sign Detection](https://github.com/globaledgesoft/AutonomousRB5/blob/main/image/stop_sign.png?raw=true)

 
