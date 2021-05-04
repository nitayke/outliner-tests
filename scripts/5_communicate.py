import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from map_roi_aligner_msgs.msg import RectangleStamped
from geometry_msgs.msg import Pose
import cv2
import numpy as np
import json
from math import pi
from constants import *

errors = []
fields = ['x', 'y', 'width', 'height', 'angle']

def callback(data):
    global j

    if "j" not in globals():
        return

    filename = data.header.frame_id
    gt = [j[filename]['x']/10, j[filename]['y']/10, j[filename]['width']/10, j[filename]['height']/10, j[filename]['angle']]
    result = [data.rectangle.centerX, data.rectangle.centerY, data.rectangle.width, data.rectangle.height, data.rectangle.theta]

    if abs(abs(gt[4] - result[4]) - pi/2) < 0.2: # 90 degrees rotated
        gt[2], gt[3] = gt[3], gt[2]
        gt[4] -= pi/2

    try:
        single_errors = [round(gt[i] - result[i], 3) for i in range(5)]
        errors.append(single_errors)
    except ZeroDivisionError:
        pass
    
    std = np.std(errors, axis=0).tolist()

    for j1 in range(5):
        if abs(single_errors[j1]) > 2*std[j1]:
            print(j1)
            print([round(i, 3) for i in gt], [round(i, 3) for i in result])
            im2 = im1 = np.zeros((SIZE, SIZE, 3), np.uint8)

            box = cv2.boxPoints(((gt[0], gt[1]), (gt[2], gt[3]), gt[4]))
            box = np.int0(box)
            cv2.drawContours(im1,[box],0,(0,0,255),2)

            box1 = cv2.boxPoints(((result[0], result[1]), (result[2], result[3]), result[4]))
            box1 = np.int0(box)
            cv2.drawContours(im2,[box1],0,(0,0,255),2)

            hori = np.concatenate((im1, im2), axis=1)

            cv2.imwrite('../trash/' + filename, hori)


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
    m.width = m.height = SIZE
    m.origin = Pose()
    m.origin.position.x = m.origin.position.y = 0

    ogrid = OccupancyGrid()
    ogrid.header.frame_id = filename
    ogrid.info = m
    ogrid.data = img.ravel().tolist()
    for i in range(len(ogrid.data)):
        ogrid.data[i] = 0 if 150 <= ogrid.data[i] <= 255 else 100
    
    pub.publish(ogrid)

# rospy.spin()

print('------ Average error: ----------')
for i in range(4):
    print(fields[i], ':', round(sum([j[i] for j in errors])/FILES_COUNT, 3), '[m]')
print('angle:', round(sum([i[4] for i in errors])/FILES_COUNT, 3), '[°]')

print('------ Standard Deviation: -----')
std = np.std(errors, axis=0).tolist()
std = [round(i, 3) for i in std]
for i in range(4):
    print(fields[i], ':', std[i], '[m]')
print(fields[4], ':', std[4], '[°]')