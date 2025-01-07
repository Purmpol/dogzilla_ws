FROM yahboomtechnology/ros-humble:3.6
# ARG USERNAME=pi
# ARG USER_UID=1000
# ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
# RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# # Create the user
# RUN groupadd --gid $USER_GID $USERNAME \
#     && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
#     && apt-get update \
#     && apt-get install -y sudo \
#     && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
#     && chmod 0440 /etc/sudoers.d/$USERNAME
# Switch from root to user
# USER $USERNAME

# Add user to video group to allow access to webcam
# RUN sudo usermod --append --groups video $USERNAME

# Update all packages
# RUN sudo apt update && sudo apt upgrade -y

# Install Git
# RUN sudo apt install -y git

# Rosdep update
# RUN rosdep update

# Source the ROS setup file
# RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ./bashrc