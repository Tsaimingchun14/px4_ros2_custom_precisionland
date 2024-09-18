# ROS2 & PX4 CustomMode Precision landing
This project is based on [this one](https://github.com/ARK-Electronics/px4_ros2_examples_ws)
and is intended for running PX4 simulation with a real raspberry pi as a companion computer and on real drone.

## Prerequisites
* Ubuntu 22.04
* ROS2 Humble

## Setup

### PC setup
Install Micro-XRCE-DDS-Agent
```bash
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```
Install QGC Daily build (custom mode only available in daily build)
```bash
cd ~
wget https://daily.qgroundcontrol.com/build/QGroundControl.AppImage
chmod +x QGroundControl.AppImage
```

Install PX4-Autopilot
```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
git checkout 8070c70
git submodule update --init --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```
(Only for simulation) To communicate with raspi connected to the same wifi, 
go in to ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS and modify this line
```uxrce_dds_client start -t udp -h 127.0.0.1 -p $uxrce_dds_port $uxrce_dds_ns```
replace 127.0.0.1 to the address of your raspberry pi

### Raspberry pi setup
Make sure you source ROS2 Humble in the terminal you are using.
```bash
source /opt/ros/humble/setup.bash 
~/.bashrc
```

Navigate to the directory you would like to place the worskpace and then run the following
```bash
git clone https://github.com/ARK-Electronics/px4_ros2_examples_ws
cd px4_ros2_ws
git submodule update --init --recursive
colcon build
source install/setup.bash 
```
## Run the example

#### Run the simulation environment on PC
To ensure message compatibility, make sure you are using the same PX4 version as shown in the video. To do this, follow these steps:

```bash
cd PX4-Autopilot/
make px4_sitl_default gz_x500
#start a new terminal
./QGroundControl.AppImage
```
Take off with the drone using the GUI


#### Launch your custom mode
I created a launch file that you can use. It currently contains only one node, so it might seem limited, but you can expand on it. The file includes three basic patterns: circle, spiral, and figure-8. These are ROS2 parameters that you can set either directly in the launch file or via command line arguments. If no pattern is specified, the default is circle.
```
#start a new terminal
MicroXRCEAgent udp4 -p 8888 
```

```
cd px4_ros2_examples_ws/
source install/setup.bash 
```
AND
```
ros2 run custom_mode custom_mode
```
OR
```
ros2 launch custom_mode custom_mode.launch.py
```
OR
```
ros2 launch custom_mode custom_mode.launch.py trajectory_type:=spiral
```
OR
```
ros2 run custom_mode custom_mode --ros-args -p trajectory_type:=figure_8

```

#### Start it from QGC
You can just start the custom node from the GUI or you can also map it to your remote control

## Setup camera on raspberry pi

### start camera 
follow this [guide](https://gaseoustortoise.notion.site/Raspberry-Pi-Camera-bc33c733eeb4417cbd5e3db027a3a429) 

### calibrate camera

follow this [guide](https://medium.com/starschema-blog/offline-camera-calibration-in-ros-2-45e81df12555)


