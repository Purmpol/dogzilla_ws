# Foxglove extensions for Dogzilla

**Requirements:**  
ROBOT: Dogzilla S2 with ROS2 Humble : https://github.com/YahboomTechnology/DOGZILLA  
Control device (PC, Tablet) : Installed Foxglove Studio + Specific extensions  


## dogzilla-gesture-cmd 
dogzilla-gesture-cmd extension: https://github.com/Purmpol/dogzilla-gesture-cmd  

### How to run  
*[terminal 1]*  
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=9090  

*[terminal 2]*  
ros2 run yahboom_publish pub_color  

*[terminal 3]*  
Clone this repos  
cd /home/pi/dogzilla_ws  
colcon build --symlink-install  
source install/setup.bash  
ros2 run my_package pub_gesture  

*[terminal 4]*  
cd /home/pi/dogzilla_ws  
source install/setup.bash  
ros2 run my_package sub_perform  

## dogzilla-voice-cmd  
dogzilla-voice-cmd extension: https://github.com/Purmpol/dogzilla-voice-cmd  

### How to run  
*[terminal 1]*  
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=9090  

*[terminal 2]*  
ros2 run voice_cmd voice_cmd  

## dogzilla-follow-waypoints  

[dogzilla-follow-waypoints](https://github.com/Purmpol/dogzilla-follow-waypoints) Foxglove Extension

### How to run
#### In Real VNC
*[terminal 1]*  
Exec a docker container of Dogzilla's ROS2
```bash
apt update
```
```bash
apt install ros-humble-rmw-cyclonedds-cpp -y
```
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```
```bash
export TURTLEBOT3_MODEL=burger
```
```bash
wget -O /root/yahboomcar_ws/install/yahboom_description/share/yahboom_description/urdf/yahboom_xgo_rviz.xacro https://raw.githubusercontent.com/Purmpol/dogzilla_ws/refs/heads/master/urdf/yahboom_xgo_rviz.xacro
```
```bash
ros2 launch bringup Navigation_bringup.launch.py
```

#### In VMWare Workstation
*[terminal 1]* 
```bash
apt update
```
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```
```bash
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
```
```bash
source ~/.bashrc
```
```bash
ros2 launch yahboom_dog_cartographer localization_imu_odom.launch.py load_state_filename:=/home/yahboom/yahboomcar_ws/maps/mymap.pbstream
```
*[terminal 2]* 
```bash
ros2 launch yahboom_dog_navigation2 navigation2.launch.py use_sim_time:=False map:=/home/yahboom/yahboomcar_ws/maps/mymap.yaml
```
*[terminal 3]* 
```bash
git clone https://github.com/Purmpol/dogzilla_ws.git
```
```bash
cd dogzilla_ws && colcon build
```
```bash
source ~/dogzilla_ws/install/setup.bash
```
```bash
ros2 run follow_waypoints follow_waypoints
```
*[terminal 4]* 
```bash
sudo apt install ros-$ROS_DISTRO-foxglove-bridge
```
```bash
source ~/dogzilla_ws/install/setup.bash
```
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=9090
```
*[terminal 5]*  
(optional) If Dogzilla keeps rotating and doesn't reach goal, try relax goal's yaw tolerance.
```bash
ros2 param set /controller_server general_goal_checker.yaw_goal_tolerance 4.0
```
