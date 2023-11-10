# OPENNI_TRACKER_ROS2

An OpenNI tracker ROS Node for ROS2. 
It broadcasts the OpenNI skeleton frames using tf2.

This is a port of the ROS1 variant located here:\
https://github.com/ros-drivers/openni_tracker

## Installation Prerequisites

The NITE library must be manually installed for openni_tracker to function. 
The two versions that are compatible with this package are 1.5.2.21 and 1.5.2.23.

NITE v1.5.2.23 can currently be downloaded from here:\
https://github.com/arnaud-ramey/NITE-Bin-Dev-Linux-v1.5.2.23

## Supported Camera

This ROS node was tested with Asus Xtion Pro under Ubuntu 22.04 running ROS Humble. 
In order to work it was neccessary to configure below parameter:

```
sudo vi /etc/openni/GlobalDefaults.ini
...
UsbInterface=2
...
```
## Setup Node ##

```bash
# run in your ros2_ws/src folder
git clone https://gitlab.com/bob-ros2/openni_tracker_ros2.git
cd ..
colcon build
. install/setup.bash
```

## ROS Node openni_tracker

### Usage

```bash
# start the node.
ros2 run openni_tracker_ros2 openni_tracker
```

## Node Parameter
> ~frame_id (string, default: camera_depth_frame)\
Anchor frame_id of skeleton frames.