#!/usr/bin/env python
import rosbag
from geometry_msgs.msg import Point
import pandas as pd

# The bag file should be in the same directory as your terminal
bag = rosbag.Bag('../../../../good3.bag')
topic = '/pointCloud'
column_names = ['x', 'y','z']
df = pd.DataFrame(columns=column_names)
num_crazyflies = 9
coordiantes = [[] for i in range(num_crazyflies)]
for i in range(num_crazyflies):
    for topic, msg, t in bag.read_messages(topics=topic):
        x = msg.points[i].x
        y = msg.points[i].y
        z = msg.points[i].z
        coordiantes[i].append([x,y,z])

    
df = pd.DataFrame({'crazyfly_{}'.format(i): coordiantes[i] for i in range(num_crazyflies)})


df.to_csv('out2.csv')