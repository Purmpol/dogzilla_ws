Requirements:  
Dogzilla S2 with ROS2 Humble : https://github.com/YahboomTechnology/DOGZILLA  
Control device with Foxglove Studio + dogzilla-gesture-cmd extension: https://github.com/Purmpol/dogzilla-gesture-cmd  


[terminal 1]
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=9090  

[terminal 2]  
ros2 run yahboom_publish pub_color  

[terminal 3]  
Clone this repos  
cd /home/pi/dogzilla_ws  
colcon build --symlink-install  
source install/setup.bash  
ros2 run my_package pub_gesture  

[terminal 4]  
cd /home/pi/dogzilla_ws  
source install/setup.bash  
ros2 run my_package sub_perform  
