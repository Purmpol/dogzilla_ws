 apt-get update
 apt install -y ros-$ROS_DISTRO-foxglove-bridge 
 apt-get install -y portaudio19-dev flac 
 pip3 install pyaudio SpeechRecognition
 echo 'source /home/pi/dogzilla_ws/install/setup.bash' >> ~/.bashrc
 ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=9090