#!/usr/bin/env python
# -*- coding: utf-8 -*-

######################################################################
#
# Software License Agreement (BSD License)
#
#  Copyright (c) 2014, ISR University of Coimbra.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the ISR University of Coimbra nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
# Author: Gonçalo S. Martins, 2014
# Inspired by an algorithm by Francisco Sales.
#####################################################################

""" 
This script creates a ROS node that takes care of a very very basic form of
collision-avoidance based exploration algorithm.
"""

from __future__ import print_function
from __future__ import division
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


# To-do:
# 1. Use whole algorithm, including deciding which way to go
# 2. Use some sort of repulsion, instead of thresholds
# 3. Make it so we only process scans once a second or so, so we're not constantly deciding
# (is this a good idea?)


class SimpleExploration:
    """ This class implements the algorithm itself.

    It keeps a few state variables, defined in the constructor, to dictate the way
    the robot moves around. Basically, what it does is analyze the range scan and
    determine which way to move.
    """
    def __init__(self):
        # Topic-related stuff
        self.pub = rospy.Publisher('cmd_vel', Twist)
        self.sub = rospy.Subscriber('scan', LaserScan, self.laserCallback)
        # Variable initialization
        self.danger = False
        self.turn_away = False
        self.danger_threshold = 0.35     # In meters
        self.angular_velocity = 0.5     # In radians, maybe
        self.linear_velocity = 0.3        # In m/s
        self.maximum_speed = 0.3
        self.minimum_speed = 0.01
        self.last_rotation_factor = 1
        self.keep_rotating = False
      
    def laserCallback(self, scan):        
        """ Callback for laser scans. This is where the magic happens. """
        # Determine the minimum distance and its index
        turning_factor = 0
        minimum_distance = 0
        minimum_index = 0
        for index, value in enumerate(scan.ranges):
            if value > scan.range_min and value < scan.range_max:
                if minimum_distance == 0 or value < minimum_distance:
                    minimum_distance = value
                    minimum_index = index
        if minimum_distance < self.danger_threshold:
            self.danger = True
            self.turn_away = False
            self.keep_rotating = True
        else:
            self.turn_away = False
            self.danger = False
            self.keep_rotating = False
        
        # Determine where to turn next
        if self.keep_rotating == True:
            turning_factor = self.last_rotation_factor
        elif minimum_index < len(scan.ranges)//2:
            turning_factor = 1
            self.last_rotation_factor = 1
        else:
            turning_factor = -1
            self.last_rotation_factor = -1

        # Issue Command
        command = Twist()
        if self.danger == True:
            command.angular.z = turning_factor*self.angular_velocity
            rospy.logdebug("Publishing z = {}.".format(command.angular.z))
        else:
            command.linear.x = self.linear_velocity*(minimum_distance)
            if command.linear.x < self.minimum_speed:
                command.linear.x = self.minimum_speed
            if command.linear.x > self.maximum_speed:
                command.linear.x = self.maximum_speed
            rospy.logdebug("Publishing x = {}, min_dist = {}.".format(command.linear.x, minimum_distance))
        
        # Publish command
        self.pub.publish(command)

if __name__ == '__main__':
    rospy.init_node('simple_exploration')
    explore = SimpleExploration()
    
    rospy.spin()
