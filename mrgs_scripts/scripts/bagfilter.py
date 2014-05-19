import rosbag

with rosbag.Bag('/home/vsantos/rosbag/2013-09-11-20-52-55_filtered.bag', 'w') as outbag:
    for topic, msg, t in rosbag.Bag('/home/vsantos/host/2013-09-11-20-52-55.bag').read_messages():
        if topic == "/robot_1/base_scan":
            msg.header.frame_id = "/laser"
            outbag.write(topic, msg, t)
        else:
            outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)
