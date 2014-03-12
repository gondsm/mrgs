#!/bin/bash
# This script is intended to make ROS's (fuerte) installation as painless and possible. As always, run at your own risk.
# This script was build for Ubuntu Precise (12.04), using the instructions available at 
#       http://wiki.ros.org/fuerte/Installation/Ubuntu
# Good luck.
#
# Gonçalo S. Martins.

# Add source
read -p "Press enter to add ROS repositories and keys, and to update apt's database."
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'

# Add key
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

# Update apt
sudo apt-get update

# Install ROS
read -p "Press enter to install ROS."
sudo apt-get install ros-fuerte-desktop-full python-rosinstall

# Source ROS
read -p "ROS installed (I think). Will now create a workspace and sanbox for you to drop this repository in. I'll even try to append a source command to your .bashrc! If you just wanted ROS, this is the time to CTRL+C."
source /opt/ros/fuerte/setup.bash

# Create workspace
mkdir ~/fuerte_workspace
rosws init ~/fuerte_workspace /opt/ros/fuerte
source ~/fuerte_workspace/setup.bash

# Create sandbox
mkdir ~/fuerte_workspace/gmartins
rosws set ~/fuerte_workspace/gmartins

# Append source command
echo 'source ~/fuerte_workspace/setup.bash' >> ~/.bashrc
