#!/bin/bash -x
# This script is intended to make this stack's installation as painless as possibile by pulling all the necessary
# code from remote repositories, compiling and installing it. Use at your own risk.
# This script assumes ROS fuerte is installed and that this script is being run from within mrgs_scripts/scripts.
#
#
# Gonçalo S. Martins.

# Install libsuitesparse, bison (for olsrd), flex (idem), from Ubuntu repositories
sudo apt-get install libsuitesparse-dev bison flex

# Pull LZ4
svn checkout http://lz4.googlecode.com/svn/trunk/ ../../mrgs_data_interface/src/lz4

# Pull and unpack olsrd
wget http://www.olsr.org/releases/0.6/olsrd-0.6.6.1.tar.bz2
tar xf olsrd-0.6.6.1.tar.bz2
rm olsrd-0.6.6.1.tar.bz2
mv olsrd-0.6.6.1/ olsrd/

# Install what we need from olsrd
cd olsrd
make
sudo make install
cd lib/txtinfo
make
sudo make install
cd ../httpinfo
make
sudo make install
cd ..
cd ..
cd ..
rm -rf olsrd

# After these steps are taken, it's important to
# sudo vim /etc/olsrd.conf
# and uncomment the line
# PlParam "port" "8080"

# Pull multimaster_experimental
svn co https://code.ros.org/svn/ros/stacks/multimaster_experimental/trunk ../../../multimaster_experimental

# Pull lse_communication
#svn co http://isr-uc-ros-pkg.googlecode.com/svn/stacks/lse_communication/trunk/lse_communication ../../../lse_communication
git clone https://bitbucket.org/gondsm/lse_communication.git ../../../lse_communication

# Pull vslam
svn co https://code.ros.org/svn/ros-pkg/stacks/vslam/trunk ../../../vslam

# Pull karto
git clone https://bitbucket.org/gondsm/karto.git ../../../karto

# Make all this stuff
rosmake sba
rosmake karto
rosmake wifi_comm
