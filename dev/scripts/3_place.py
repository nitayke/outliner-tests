import os
import cv2
import numpy as np
from random import randint
from math import radians, cos, sin
import yaml
from constants import *

def rotate_image(image, angleInDegrees):
    h, w = image.shape[:2]
    img_c = (w / 2, h / 2)

    rot = cv2.getRotationMatrix2D(img_c, angleInDegrees, 1)

    rad = radians(angleInDegrees)
    sin1 = sin(rad)
    cos1 = cos(rad)
    b_w = int((h * abs(sin1)) + (w * abs(cos1)))
    b_h = int((h * abs(cos1)) + (w * abs(sin1)))

    rot[0, 2] += ((b_w / 2) - img_c[0])
    rot[1, 2] += ((b_h / 2) - img_c[1])

    outImg = cv2.warpAffine(image, rot, (b_w, b_h), flags=cv2.INTER_LINEAR, borderValue=(255,255,255))
    return outImg

directory = "../3_after_resize/"

for filename_int in range(1, FILES_COUNT + 1):
    filename = str(filename_int) + '.jpg'
    s_img = cv2.imread(directory + filename)
    l_img = np.zeros((IMAGE_SIZE, IMAGE_SIZE, 3), np.uint8) # blank image
    l_img.fill(255)
    angle = randint(0, 90)
    s_img = rotate_image(s_img, angle)
    height, width, _ = s_img.shape
    while width > MAX_BUILDING_SIZE or height > MAX_BUILDING_SIZE:
        height, width = int(height/2), int(width/2)
        s_img = cv2.resize(s_img, (width, height), interpolation=cv2.INTER_AREA)
        # angle = randint(0, 90)
        # s_img = rotate_image(s_img, angle)
    while width < MIN_BUILDING_SIZE or height < MIN_BUILDING_SIZE:
        height, width = height*2, width*2
        s_img = cv2.resize(s_img, (width, height), interpolation=cv2.INTER_AREA)
        # angle = randint(0, 90)
        # s_img = rotate_image(s_img, angle)

    x_offset = randint(0, (IMAGE_SIZE-width))
    y_offset = randint(0, IMAGE_SIZE-height)
    l_img[y_offset:y_offset+height, x_offset:x_offset+width] = s_img
    
    cv2.imwrite('../4_final/' + filename, l_img)
    print(filename)