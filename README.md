# Field AI interface for legged perceptive Control


## Dependency

### OCS2
```
# Clone OCS2
git clone git@github.com:SFakoorian0111/ocs2.git
```
#### Dependency
```
# Clone pinocchio
git clone --recurse-submodules https://github.com/leggedrobotics/pinocchio.git
# Clone hpp-fcl
git clone --recurse-submodules https://github.com/leggedrobotics/hpp-fcl.git
# Clone ocs2_robotic_assets
git clone https://github.com/leggedrobotics/ocs2_robotic_assets.git
# Install dependencies
sudo apt install liburdfdom-dev liboctomap-dev libassimp-dev
```
### elevation_map
```
# Clone elevation mapping
git clone git@github.com:ANYbotics/elevation_mapping.git
# Clone elevation mapping cupy
git clone git@github.com:leggedrobotics/elevation_mapping_cupy.git
```
#### Dependency
```
# Clone grid map
git clone git@github.com:ANYbotics/grid_map.git
# Clone kindr
git clone git@github.com:ANYbotics/kindr.git
# Clone kindr_ros
git clone git@github.com:ANYbotics/kindr_ros.git
# Clone message logger
git clone git@github.com:ANYbotics/message_logger.git
# Clone realsense_gazebo_plugin (Note: for simulation only)
git clone git@github.com:pal-robotics/realsense_gazebo_plugin.git
#[realsense2_description](https://github.com/IntelRealSense/realsense-ros/tree/ros2-development/realsense2_description)
sudo apt-get install ros-$ROS_DISTRO-realsense2-description
```

### legged_control
```
# Clone legged_control
git clone git@github.com:qiayuanl/legged_control.git
```
### legged_control_perceptive
```
# Clone legged_control_perceptive
git clone git@github.com:qiayuanl/legged_perceptive.git
```
