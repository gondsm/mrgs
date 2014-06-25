#!/usr/bin/env python

from __future__ import print_function
from __future__ import division
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


# To-do:
# 1. Use whole algorithm, including deciding which way to go
# 2. Port this into the scripts package, it doesn't deserve a whole package
# 3. Make it so we only process scans once a second or so, so we're not constantly deciding
# (is this a good idea?)

class SimpleExploration:
  # Constructor
  def __init__(self):
    # Topic-related stuff
    self.pub = rospy.Publisher('cmd_vel', Twist)
    self.sub = rospy.Subscriber('scan', LaserScan, self.laserCallback)
    # Variable initialization
    self.danger = False
    self.danger_threshold = 0.3 # In meters
    self.angular_velocity = 0.2 # In m/s, somehow
    self.linear_velocity = 0.1 # In m/s
    
  # Callback for laser scans. This is where the magic happens.
  def laserCallback(self, scan):
    # Start counting time, for logging purposes
    init = rospy.get_rostime()
    
    # Determine if we're in danger
    self.danger = False
    for value in scan.ranges:
      if value > scan.range_min and value < scan.range_max and value < self.danger_threshold:
        self.danger = True
        break
    
    # Determine where we should go next, either forward or rotate
    command = Twist()
    if self.danger == True:
      command.angular.z = self.angular_velocity
    else:
      command.linear.x = self.linear_velocity
      
    # Publish command
    self.pub.publish(command)
    
    # Report performance
    rospy.loginfo("We took {} seconds to process the current scan.".format((rospy.get_rostime() - init)))

if __name__ == '__main__':
  rospy.init_node('simple_exploration')
  explore = SimpleExploration()
  
  rospy.spin()
