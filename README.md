# TO-DO

This package is currently under heavy re-work. This branch is expected to be very unstable for a while.

- [ ] Re-work installation mechanism (`rosdep install mrgs`)
- [X] Re-work topics to be compatibel with `multimaster_fkie`
- [ ] Add `multimaster_fkie` to launch files
- [ ] Add launch files to launch multiple systems with node name suffixes
- [ ] Test with new datasets
- [X] Update README with new instructions

mrgs
===
A Cooperative SLAM Framework with Efficient Information Sharing over Mobile Ad Hoc Networks.

This work was developed by [Gonçalo Martins](http://ap.isr.uc.pt/?w=people_information&ID=152) in the context of his M.Sc. work:

[Martins, Gonçalo S. (2014). A Cooperative SLAM Framework with Efficient Information Sharing over Mobile Ad Hoc Networks. M.Sc. Dissertation, University of Coimbra, Portugal.](http://mrl.isr.uc.pt/archive/GMartins_dissertation_final.pdf)

## Introduction
This software solution aims to transparently enable any Single-Robot SLAM system to perform Multi-Robot SLAM. It is implemented using the [ROS](http://ros.org) cummunication framework. It is divided into five different packages.

To achieve our goal, this system runs on top of any existing SLAM system that conforms to ROS's usual standards for SLAM (e.g. the usage of the OccupancyGrid message to transmit occupancy grids), and enables it to communicate with nearby SLAMming robots in order to build a joint representation of the environment.

This is an updated version of the package that runs on ROS Melodic (and, I believe, newer). It no longer depends on some old unmaintained packages, and should still remain mildly useful.

## mrgs: A Practical Guide

### Requirements

Everything you need should be covered by `rosdep install mrgs`. Still, for reference, this package relies on OpenCV (which you can install with ROS) and [LZ4](github.com/lz4/lz4). It is also necessary that your robots have [multimaster_fkie](http://wiki.ros.org/multimaster_fkie) properly configured. This package includes a configuration that is used in our launch files, but actually configuring your network, hostnames, etc, falls out of our scope.

### Running the system
The system can run in one of two different modes: centralized and distributed. The distributed mode runs the full mrgs system on every robot, and the centralized mode relies on a non-mapping computer to receive the maps and do all the heavy processing.

The launching of the system is taken care of by using the provided launch files.

#### Distributed (normal) mode:

Just launch

```
roslaunch mrgs distributed_node.launch
```

on each of your robots.

#### Centralized mode:
On the central machine:

```
roslaunch mrgs_scripts central_node.launch
```

On the remaining machines:

```
roslaunch mrgs_scripts mapper_node.launch
```    

#### Something else

You can mix and match the centralized and distributed nodes to fit your particular team. Just make sure that every multimaster_fkie is up and running correctly.

## Overview
The code is divided into four packages, described below, with an additional package holding scripts, launch files, bags and the like.

### mrgs_data_interface
This package contains a node abstracts away all the network-related issues. It propagates maps and transforms into it and receives them back. It publishes a vector of maps (the latest maps the current robot has, including its own) into the...

### mrgs_complete_map
This package contains a node that takes care of the process of fusing maps one by one. It does so by incrementally building a tree of maps. This way, we only have to re-fuse maps when we have new information. By keeping all the intermediate steps between the local maps and the final fused map, we can reuse this information so that we do not need to re-fuse everything every time a new map is received. This node controls another node, that takes care of the merging itself, called...

### mrgs_alignment
This package contains a node that receives two occupancy grids and returns a single, fused one, a pair of transforms (from each of the received maps to the fused map) and a coefficient that varies between 0 and 1 and is meant to indicate how sure we are of our fusion.

The transforms that this node generates, as well as the ones received from other robots, are propagated into a node located in the package called...

### mrgs_auxiliary_nodes
This package contains two nodes. The remote_nav_node receives transforms from the complete_map and data_interface nodes and publishes them, correctly normalized, prefixed and timed, into [tf](http://wiki.ros.org/tf).

Finally, this package contains the map_dam_node, that is responsible for listening to the /map topic (or whatever topic your SLAM technique is publishing into) and for deciding whether or not this map should enter our system.




