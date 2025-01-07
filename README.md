For Dogzilla S2 with ROS2 Humble

[terminal 1]
Clone this repos
cd /home/pi/dogzilla_ws
colcon build --symlink-install 
source install/setup.bash
ros2 run my_package pub_gesture

[terminal 2]
cd /home/pi/dogzilla_ws
source install/setup.bash
ros2 run my_package sub_perform
