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

    for i in range(min([len(line[0]) for line in img])):
        for j in range(len(img)):
            if not img[j][-i].any():
                right = j, len(img[i]) - i
                break
        else:
            continue
        break

    f.write(filename + ': (' + str(bottom[0] - top[0]) + ', ' + str(right[1] - left[1]) + ')\n')
    # format: "<filename>: (<height>, <width>)"
    print filename + ' ' + str(bottom[0] - top[0]) + ' ' + str(right[1] - left[1])


f.close()