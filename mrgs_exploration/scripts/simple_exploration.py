#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class SimpleExploration:
  # Constructor
  def __init__(self):
    self.pub = rospy.Publisher('cmd_vel', Twist)
    self.sub = rospy.Subscriber('scan', LaserScan, self.laserCallback)
    self.danger = False
    self.danger_threshold = 0.1 # In meters
    
  # Callback for laser scans. This is where the magic happens.
  def laserCallback(self, scan):
    # Determine if we're in danger
    for value in scan.ranges:
      if value > scan.range_min and value < scan.range_max and value < danger_threshold:
        self.danger = True
        break
    command = Twist()
    
    # Determine where we should go next, either forward or rotate
    if self.danger == True:
      command.angular.z = 0.2
    else:
      command.linear.x = 0.1
      
    # Publish command
    self.pub.publish(command)

if __name__ == '__main__':
  rospy.init_node('simple_exploration')
  explore = SimpleExploration()
  
  rospy.spin()
