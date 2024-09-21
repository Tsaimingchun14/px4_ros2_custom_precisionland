# ROS2 & PX4 CustomMode Precision landing
This project aims to implement a precision landing system on a PX4 based drone.
It utilize the Apriltag marker as the landing target and create a custom flight mode that implements a landing algorithm.
#### This guide includes 
1. Simulation process on a single computer 
2. Setting up raspberry pi 4 as a companion computer for a real drone
3. Precision landing on a real drone with raspberry pi (coming soon)

## Simulation on a single computer

### Prerequisites

* Ubuntu 22.04
* ROS2 Humble

### Setup

1. Install PX4-Autopilot

```bash=
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
git checkout 9ca0630376fca98396943c065034d3cacae34746
# At the time of testing, the latest commit had some problem.
git submodule update --init --recursive
bash ./Tools/setup/ubuntu.sh
# This will install gz-harmonic for us, which is the simulation tool.
```
2. Install QGC Daily build (custom mode only available in daily build)
```bash=
cd ~
wget https://daily.qgroundcontrol.com/build/QGroundControl.AppImage
chmod +x QGroundControl.AppImage
```
3. Install ros-gz
In order to bridge the message in gazebo sim to our ros environment, we need to install ros-gz
According to this [guide](https://gazebosim.org/docs/harmonic/ros_installation/#-gazebo-harmonic-with-ros-2-humble-iron-or-rolling-use-with-caution-) :
```basg=
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
apt-get install ros-humble-ros-gzharmonic
```
The following setups are for the companion computer. Since we are running simulation on a single machine, we will set up on the same computer.


4. Install Micro-XRCE-DDS-Agent
```bash=
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

5. Install Apriltag library
```bash=
cd ~
git clone https://github.com/AprilRobotics/apriltag.git
cd apriltag
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target install
```
6. Install image pipeline
We need this package to recitify the camera image
```bash=
sudo apt install ros-humble-image-pipeline
```
7. Clone and build the main workspace that contains the main precision landing package and other curcial packages.
```bash=
cd ~
git clone https://github.com/Tsaimingchun14/px4_ros2_custom_precisionland.git
git submodule update --init --recursive
# This we checkout the submodules to the specific commit
# you should make sure PX4_msgs is on commit 620ed0f and px4-ros2-interface-lib is on c75a5f5.
# These two package along with PX4 highly depend on one another. Be carefull when you want to update these packages.

#Copy our custom luanch file and config file into apriltag_ros
cd ~/px4_ros2_custom_precisionland
cp resource/tags_41h12.yaml src/apriltag_ros/cfg
cp resource/v4l2_41h12_simulation.launch.yml src/apriltag_ros/launch
cp resource/v4l2_41h12.launch.yml src/apriltag_ros/launch

colcon build
# After this is done once, later if you want to rebuild the packages you've modified
# you can do `colcon build --packages-select ${pkg1} ${pkg2}...` to avoid rebuilding other unmodified packages.
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
echo "source ~/px4_ros2_custom_precisionland/install/setup.bash" >> ~/.bashrc
```

### Run the simulation
Extra steps only for simulation
1. `UXRCE_DDS_SYNCT` : When enabled, uxrce_dds_client will synchronize the timestamps of the incoming and outgoing messages measuring the offset between the Agent OS time and the PX4 time.
For simulation, since gazebo time factor is never 1, it can never be synced, so we need to disable it and use gazebo time as the only source of clock.
Go to ~/PX4-Autopilot/src/modules/uxrce_dds_client.module.yaml and set ```UXRCE_DDS_SYNCT```default to 0
2. The simualtion world we are using is the PX4 built in aruco world. We need to replace the tag with Apriltag's tagStandard41h12:0 (located in /resource)
Go to `PX4-Autopilot/Tools/simulation/gz/models/arucotag` and replace the `aruco_tag.png` with our apriltag image and remember to name it `aruco_tag.png`.
Modify the tag size in 
`PX4-Autopilot/Tools/simulation/gz/models/arucotag/model.sdf` to `0.17 0.17`
3. Since I want to simulate the camera I have for now, go to 
`PX4-Autopilot/Tools/simulation/gz/models/mono_cam/model.sdf` and change `horizontal_fov` to `0.866` and `width` `height` to `640` `480`.

Now we can finally run the simulation.
Run the following commands in separate terminals
```bash=
cd PX4-Autopilot
make px4_sitl gz_x500_mono_cam_down_aruco

cd ~
./QGroundControl-x86_64.AppImage

MicroXRCEAgent udp4 -p 8888

ros2 launch apriltag_ros v4l2_41h12_simulation.launch.yml

ros2 launch precision_land precision_land_simulation.launch.py
```

Now you can takeoff and switch to PrecisionlandCustom mode in QGC

## Setting up raspberry pi 4

In this section we

### Prerequisites
* A raspberry pi 4
* Ubuntu 22.04
* ROS2 Humble
* A raspberry pi camera
* Apriltag tagStandard41h12:0 (17cm*17cm) printed on a paper

### Setup
1. Setup camera 
Follow this [guide](https://gaseoustortoise.notion.site/Raspberry-Pi-Camera-bc33c733eeb4417cbd5e3db027a3a429) to make sure your camera can operate correctly with `v4l2_camera` package and `/image_raw` is being published.

2. Follow step 4 to 7 in the Setup section from the previous part.

3. Calibrate camera
Follow this [guide](https://medium.com/starschema-blog/offline-camera-calibration-in-ros-2-45e81df12555) to get the calibration file and provide it to `v4l2_camera`. 
Note: calibration tool includes GUI interface, you may need xforwarding depending on your pc and raspberry pi setup.


### Apriltag test with real camera
```bash=
ros2 launch apriltag_ros v4l2_41h12.launch.yml
```

### Raspberry pi communication test with PX4(on your PC)

Connect your pc and raspberry pi to the same wifi and go in to `~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS`. Modify this line
`uxrce_dds_client start -t udp -h 127.0.0.1 -p $uxrce_dds_port $uxrce_dds_ns`,
replace 127.0.0.1 with the address of your raspberry pi.

* on PC
```bash=
cd PX4-Autopilot
make px4_sitl_default gz_x500
```
* on rpi
```bash=
MicroXRCEAgent udp4 -p 8888
```
Now you should see PX4 topics being published on raspberry pi.

## Precision landing on a real drone
(coming soon)







