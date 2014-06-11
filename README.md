mrgs
===
A general communication framework for Multi-Robot SLAM based on Occupancy Grid merging.  
This work was developed by Gonçalo Martins in the context of his MSc work.

## Introduction
This software solution aims to transparently enable any Single-Robot SLAM system to perform Multi-Robot SLAM. It is implemented using the [ROS](http://ros.org) cummunication framework. It is divided into five different packages.

<!-- Imagem! -->

To achieve our goal, this system runs on top of any existing SLAM system that conforms to ROS's usual standards for SLAM (e.g. the usage of the OccupancyGrid message to transmit occupancy grids), and enables it to communicate with nearby SLAMming robots in order to build a joint representation of the environment.

## mrgs: A Practical Guide

### Requirements
1. This system runs on top of ROS Fuerte, which requires that the machine you use is running Linux, preferrably Ubuntu 12.04. Other flavours of Ubuntu may or may not work with ROS.
2. You must already have a working SLAM system, since this system does not include a SLAM solution. A commonly used SLAM package is [GMapping](http://wiki.ros.org/gmapping).
3. The system itself depends on a few other things, which are detailed on the setup script (mrgs_scripts/scripts/setup.sh), which brings us to...

### Setup
Setting up the system is as simple as running the setup.sh bash script that exists in the scripts folder of the mrgs_scripts package. Please do not do this blindly! The script has instructions on the top, read them carefully. This script downloads and installs everything you should need in order to run this on top of your pre-existing SLAM solution. Running the script is usually accomplished by:

    $roscd mrgs_scripts/scripts
    $./setup.sh
    
or, alternatively, if you want to see the script (which you should):

    $roscd mrgs_scripts/scripts
    $nano setup.sh
    
Again, **do no do this blindly**, take a look at the script before executing it. In this folder you will find other useful goodies, like the ip_setup python script, and a script that takes your system from freshly installed to having a working ROS Fuerte system.

### Running the system
The system can run in one of two different modes: centralized and distributed. The distributed mode runs the full mrgs system on every robot, and the centralized mode relies on a non-mapping computer to receive the maps and do all the heavy processing.

The launching of the system is taken care of by using the provided launch files.

#### Distributed (normal) mode:

    $roslaunch mrgs_scripts distributed_node.launch interface:=wlan0
    
Interface is usually wlan0, eth0 or something similar, substitute wlan0 with whatever interface OLSRd is using.

#### Centralized mode:
On the central machine:

    $roslaunch mrgs_scripts central_node.launch
    
On the remaining machines:

    $roslaunch mrgs_scripts transmitter_node.launch interface:=wlan0
    
Same rules as before for the interface argument.

## Stack Overview
The code is divided into four packages, with an additional package holding scripts, launch files, bags and the like.

### mrgs_data_interface
This package contains a node abstracts away all the network. It propagates maps and transforms into it and receives them back. It publishes a vector of maps (the latest maps the current robot has, including its own) into the...

### mrgs_complete_map
This package contains a node that takes care of the process of fusing maps one by one. It does so by incrementally building a tree of maps. This way, we only have to re-fuse maps when we have new information. By keeping all the intermediate steps between the local maps and the final fused map, we can reuse this information so that we do not need to re-fuse everything every time a new map is received. This node controls another node, that takes care of the merging itself, called...

### mrgs_alignment
This package contains a node that receives two occupancy grids and returns a single, fused one, a pair of transforms (from each of the received maps to the fused map) and a coefficient that varies between 0 and 1 and is meant to indicate how sure we are of our fusion.

The transforms that this node generates, as well as the ones received from other robots, are propagated into a node located in the package called...

### mrgs_auxiliary_nodes
This package contains two nodes. The remote_nav_node receives transforms from the complete_map and data_interface nodes and publishes them, correctly normalized, prefixed and timed, into [tf](http://wiki.ros.org/tf).

Finally, this package contains the map_dam_node, that is responsible for listening to the /map topic (or whatever topic your SLAM technique is publishing into) and for deciding whether or not this map should enter our system.




