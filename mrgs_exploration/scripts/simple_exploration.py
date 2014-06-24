#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

pub = rospy.Publisher('cmd_vel', Twist)

def laserCallback(scan):
  return

if __name__ == '__main__':
  rospy.init_node('simple_exploration')
  sub = rospy.Subscriber('scan', LaserScan, laserCallback)
  command = Twist()
  command.angular.z = 1
  pub.publish(command)
  rospy.spin()
