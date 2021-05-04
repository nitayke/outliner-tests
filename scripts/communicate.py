#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from map_roi_aligner_msgs.msg import RectangleStamped
from geometry_msgs.msg import Pose
import cv2
import numpy as np
import json
from math import pi

def round1(num):
    return round(num, 2)

def callback(data):
    global j

    filename = data.header.frame_id
    rect = [[j[filename]['x'], j[filename]['y']], [j[filename]['width'], j[filename]['height']], j[filename]['angle']]
    
    result = [[round1(10*data.rectangle.centerX), round1(10*data.rectangle.centerY)], [round1(10*data.rectangle.width),
     round1(10*data.rectangle.height)], round1(data.rectangle.theta)]


    if abs(abs(rect[2] - result[2]) - pi/2) < 0.1:
        rect[1][0], rect[1][1] = rect[1][1], rect[1][0]
        rect[2] = result[2]

    # im2 = im1 = np.zeros((250, 250, 3), np.uint8)

    # box = cv2.boxPoints(rect)
    # box = np.int0(box)
    # cv2.drawContours(im1,[box],0,(0,0,255),2)

    # box1 = cv2.boxPoints(result)
    # box1 = np.int0(box)
    # cv2.drawContours(im2,[box1],0,(0,0,255),2)

    # hori = np.concatenate((im1, im2), axis=1)

    # cv2.imshow('name', hori)
    # cv2.waitKey()

    
    print(rect)
    print(result)

    print()

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

    # box = cv2.boxPoints(rect)
    # box = np.int0(box)
    # cv2.drawContours(im1,[box],0,(0,0,255),2)
    # cv2.imshow('f', im1)
    # cv2.waitKey()

    # d = np.reshape(np.array(ogrid.data, dtype=np.uint8), (250, 250))

rospy.spin()