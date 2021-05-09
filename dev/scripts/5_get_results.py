import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from map_roi_aligner_msgs.msg import RectangleStamped
from geometry_msgs.msg import Pose
import cv2
import numpy as np
import json
from math import pi
from constants import *

errors = {} # {<filename>: [[<e_x>, <e_y>, <e_width>, <e_height>, <e_angle>], data]}

def callback(data):
    global j
    if "j" not in globals():
        return

    filename = data.header.frame_id
    gt = [j[filename]['x'], j[filename]['y'], j[filename]['width'], j[filename]['height'], j[filename]['angle']]
    result = [data.rectangle.centerX, data.rectangle.centerY, data.rectangle.width, data.rectangle.height, data.rectangle.theta]

    if abs(abs(gt[4] - result[4]) - pi/2) < 0.2: # 90 degrees rotated
        gt[2], gt[3] = gt[3], gt[2]
        gt[4] -= pi/2

    single_errors = [gt[i] - result[i] for i in range(5)]
    errors[filename] = [single_errors, result]
    
    print(filename)


rospy.init_node('test_img')

pub = rospy.Publisher('map_handler/out/debug_map', OccupancyGrid, queue_size=100)
sub = rospy.Subscriber('map_roi/out/rectangle/center_size_rot', RectangleStamped, callback, queue_size=100)
rospy.sleep(1.)

file = open('../gt.json')
txt = file.read()
j = json.loads(txt)
file.close()

for filename in j.keys():
    img = cv2.imread('../4_final/' + filename)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    m = MapMetaData()
    m.resolution = 0.1
    m.width = m.height = IMAGE_SIZE
    m.origin = Pose()
    m.origin.position.x = m.origin.position.y = 0

    ogrid = OccupancyGrid()
    ogrid.header.frame_id = filename
    ogrid.info = m
    ogrid.data = img.ravel().tolist()
    for i in range(len(ogrid.data)):
        ogrid.data[i] = 0 if 150 <= ogrid.data[i] <= 255 else 100
    
    pub.publish(ogrid)

rospy.spin() # TODO: stop it automatically

file = open('../results.json', 'w')
file.write(json.dumps(errors, indent=3))
file.close()
