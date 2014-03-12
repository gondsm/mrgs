#!/bin/bash -x
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
sudo apt-get install ros-fuerte-desktop-full
