import os
import cv2
import numpy as np
from time import time
from random import choice, randint
import math

BIG_SIZE = 2500

def rotate_image(image, angleInDegrees):
    h, w = image.shape[:2]
    img_c = (w / 2, h / 2)

    rot = cv2.getRotationMatrix2D(img_c, angleInDegrees, 1)

    rad = math.radians(angleInDegrees)
    sin = math.sin(rad)
    cos = math.cos(rad)
    b_w = int((h * abs(sin)) + (w * abs(cos)))
    b_h = int((h * abs(cos)) + (w * abs(sin)))

    rot[0, 2] += ((b_w / 2) - img_c[0])
    rot[1, 2] += ((b_h / 2) - img_c[1])

    outImg = cv2.warpAffine(image, rot, (b_w, b_h), flags=cv2.INTER_LINEAR, borderValue=(255,255,255))
    return outImg

directory = "/home/lab1/nitay/buildings/dataset/"

for filename in os.listdir(directory):
    s_img = cv2.imread(directory + filename)
    l_img = np.zeros((BIG_SIZE, BIG_SIZE, 3), np.uint8) # blank image
    l_img.fill(255)

    s_img = rotate_image(s_img, randint(0, 360))
    height, width, _ = s_img.shape
    while BIG_SIZE < width or BIG_SIZE < height:
        s_img = cv2.resize(s_img, (int(0.5*width), int(0.5*height)), interpolation=cv2.INTER_AREA)
        s_img = rotate_image(s_img, randint(0, 360))
        height, width, _ = s_img.shape
        print height, width
    x_offset = randint(0, (BIG_SIZE-width))
    y_offset = randint(0, BIG_SIZE-height)
    l_img[y_offset:y_offset+height, x_offset:x_offset+width] = s_img

    cv2.imwrite('/home/lab1/nitay/buildings/big_images/' + filename, cv2.resize(l_img, (int(l_img.shape[1]/10), int(l_img.shape[0]/10)), interpolation=cv2.INTER_AREA))
    print filename
