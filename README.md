# custom_ros_lib

## Install livox_ros_driver2 for mid360 LiDAR
- [Livox SDK2](https://github.com/Livox-SDK/Livox-SDK2)
- [Livox ros driver2](https://github.com/Livox-SDK/livox_ros_driver2)

## livox2pcd
```
mkdir -p ~/workspace/src && \
mkdir -p ~/workspace/data/pcd && \
mkdir -p ~/workspace/data/timestamp && \
cd ~/workspace/src

source /opt/ros/${rosversion}/setup.sh
catkin_init_workspace

git clone https://github.com/Phw9/custom_ros_lib.git
git clone https://github.com/Livox-SDK/livox_ros_driver2.git

cd livox_ros_driver2
./build.sh ROS1

after we can use catkin_make in workspace
```
