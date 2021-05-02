# step 4

import json
import cv2
import os
import numpy as np

directory = "../final/"
f = open('../rectangles', 'w+')
for filename in os.listdir(directory):
    l_img = cv2.imread(directory + filename)
    gray = cv2.cvtColor(l_img, cv2.COLOR_BGR2GRAY)
    thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]

    cnts = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    im = np.zeros((250, 250, 3), np.uint8) # blank image
    if len(cnts) > 1:
        tmp = cnts[0]
        for i in cnts[1:]:
            tmp = np.append(tmp, i, axis=0)
        cnts = [tmp]
    cv2.drawContours(im, cnts, 0, (0,255,0), 3)

    ((x, y), (width, height), angle) = cv2.minAreaRect(cnts[0])
    (x, y, width, height, angle) = (round(x, 2), round(y, 2), round(width, 2), round(height, 2), round(angle, 2))
    d = {filename: {'x': x, 'y': y, 'width': width, 'height': height, 'angle': angle}}
    f.write(json.dumps(d, indent=4) + "\n")
    print(filename)

f.close()