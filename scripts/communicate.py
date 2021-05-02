#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from map_roi_aligner_msgs.msg import RectangleStamped
from geometry_msgs.msg import Pose
import cv2
import numpy as np
import json

def callback(data): # not running yet
    print(data.rectangle)

rospy.init_node('test_img')

pub = rospy.Publisher('map_handler/out/debug_map', OccupancyGrid, queue_size=100)
sub = rospy.Subscriber('map_roi/out/rectangle/center_size_rot', RectangleStamped, callback, queue_size=100)
rospy.sleep(1.)

file = open('../rectangles.json')
txt = file.read()
j = json.loads(txt)
file.close()

for filename in j.keys():
    img = cv2.imread('../final/' + filename)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    m = MapMetaData()
    m.resolution = 0.1
    m.width = m.height = 250
    m.origin = Pose()
    m.origin.position.x = m.origin.position.y = 0

    ogrid = OccupancyGrid()
    ogrid.header.frame_id = filename
    ogrid.info = m
    ogrid.data = img.ravel().tolist()
    for i in range(len(ogrid.data)):
        ogrid.data[i] = 0 if 150 <= ogrid.data[i] <= 255 else 100
    
    pub.publish(ogrid)

    
    # d = np.reshape(np.array(ogrid.data, dtype=np.uint8), (250, 250))

rospy.spin()