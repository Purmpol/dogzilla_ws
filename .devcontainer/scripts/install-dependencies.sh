 apt-get update
 apt install -y ros-$ROS_DISTRO-foxglove-bridge 
 apt-get install -y portaudio19-dev flac 
 pip3 install pyaudio SpeechRecognition
 sudo apt install ros-humble-nav2-msgs
 sudo apt install ros-humble-rmw-cyclonedds-cpp
 echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
 echo 'source /home/pi/dogzilla_ws/install/setup.bash' >> ~/.bashrc
 echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
 export TURTLEBOT3_MODEL=burger
 wget -O /root/yahboomcar_ws/install/yahboom_description/share/yahboom_description/urdf/yahboom_xgo_rviz.xacro https://raw.githubusercontent.com/Purmpol/dogzilla_ws/refs/heads/master/urdf/yahboom_xgo_rviz.xacro
