import os
import cv2
import numpy as np
from random import randint
from math import radians, cos, sin
import yaml

BIG_SIZE = 250

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

    outImg = cv2.warpAffine(image, rot, (b_w, b_h), flags=cv2.INTER_NEAREST, borderValue=(255,255,255))
    return outImg

directory = "/home/lab1/nitay/buildings/small_dataset/"
file = open('./sizes')
corners = file.read()
file.close()

sizes = yaml.load(corners)
file = open('./coordinates', 'w+')

for filename in os.listdir(directory):
    s_img = cv2.imread(directory + filename)
    l_img = np.zeros((BIG_SIZE, BIG_SIZE, 3), np.uint8) # blank image
    l_img.fill(255)
    angle = randint(0, 90)
    s_img = rotate_image(s_img, angle)
    height, width, _ = s_img.shape
    while BIG_SIZE < width or BIG_SIZE < height:
        s_img = cv2.resize(s_img, (int(0.5*width), int(0.5*height)), interpolation=cv2.INTER_AREA)
        angle = randint(0, 90)
        s_img = rotate_image(s_img, angle)
        height, width, _ = s_img.shape
        print height, width
        sizes[filename][0], sizes[filename][1] = sizes[filename][0] // 2, sizes[filename][1] // 2
    x_offset = randint(0, (BIG_SIZE-width))
    y_offset = randint(0, BIG_SIZE-height)
    l_img[y_offset:y_offset+height, x_offset:x_offset+width] = s_img
    
    cv2.imwrite('/home/lab1/nitay/buildings/big_images/' + filename, l_img)
    print filename, angle
