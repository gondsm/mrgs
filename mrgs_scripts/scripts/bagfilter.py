# This script converts bagfiles that already existed in the project so that they may be played back into my system.
import rosbag
import re

# Configure here:
input = '/home/vsantos/host/rosbag_filtered/1.bag'
output = '/home/vsantos/test/1_filtered.bag'
#prefix_to_remove = "/robot_0"

# Prefix is now removed via regular expression
p = re.compile('/robot_./base_scan')

# Process bagfile
with rosbag.Bag(output, 'w') as outbag:
    for topic, msg, t in rosbag.Bag(input).read_messages():
        #if topic == prefix_to_remove + "/base_scan":
        if p.match(topic):
            #print('Detected! ' + topic)
	    msg.header.frame_id = "/laser"
            outbag.write(topic, msg, t)
        #else:
            outbag.write(topic, msg, t)
