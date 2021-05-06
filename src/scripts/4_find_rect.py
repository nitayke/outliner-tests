import json
import cv2
from math import radians
import os
import numpy as np
from constants import *

directory = "../4_final/"
d = {}

for filename_int in range(1, FILES_COUNT + 1):
    filename = str(filename_int) + '.jpg'
    l_img = cv2.imread(directory + filename)
    gray = cv2.cvtColor(l_img, cv2.COLOR_BGR2GRAY)
    thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]

    cnts = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    im = np.zeros((IMAGE_SIZE, IMAGE_SIZE, 3), np.uint8) # blank image
    if len(cnts) > 1:
        tmp = cnts[0]
        for i in cnts[1:]:
            tmp = np.append(tmp, i, axis=0)
        cnts = [tmp]
    cv2.drawContours(im, cnts, 0, (0,255,0), 3)

    ((x, y), (width, height), angle) = cv2.minAreaRect(cnts[0])
    (x, y, width, height, angle) = (round(x/10, 3), round(y/10, 3), round(width/10, 3), round(height/10, 3), round(angle, 3))
    d[filename] = {'x': x, 'y': y, 'width': width, 'height': height, 'angle': radians(angle)}
    print(filename)

    # ------- for drawing rectangles on it: --------

    # box = cv2.boxPoints(((x, y), (width, height), angle))
    # box = np.int0(box)
    # cv2.drawContours(l_img, [box], 0, (0, 0, 255), 3)

    # cv2.imwrite('../final_with_rects/' + filename, l_img)
    

f = open('../gt.json', 'w')
f.write(json.dumps(d, indent=3))
f.close()