import os
import cv2
import numpy as np
from time import time

directory = "/home/lab1/nitay/buildings/dataset/"
f = open('./corners', 'w+')
for filename in os.listdir(directory):
    img = cv2.imread(directory + filename)
    for i in range(len(img)):
        for j in range(len(img[i])):
            if not img[i][j].any():
                top = i, j
                break
        else:
            continue
        break

    for i in range(len(img)):
        for j in range(len(img[-i])):
            if not img[-i][j].any():
                bottom = len(img)-i, j
                break
        else:
            continue
        break

    for i in range(len(img[0])):
        for j in range(len(img)):
            if not img[j][i].any():
                left = j, i
                break
        else:
            continue
        break

    for i in range(len(img[0])):
        for j in range(len(img)):
            if not img[j][-i].any():
                right = j, len(img[i]) - i
                break
        else:
            continue
        break

    f.write(str(top) + str(bottom) + str(right) + str(left) + '\n')
    print top, bottom, right, left, filename

f.close()