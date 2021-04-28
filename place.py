import os
import cv2
import numpy as np
from time import time
from random import choice, randint

def rotate_image(image, angle):
    image_center = tuple(np.array(image.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    return result

directory = "/home/lab1/nitay/buildings/dataset/"

for filename in os.listdir(directory):
    s_img = cv2.imread(directory + filename)
    height, width, _ = s_img.shape
    l_img = np.zeros((5000, 5000, 3), np.uint8) # blank image
    l_img.fill(255)

    s_img = rotate_image(s_img, randint(0, 360))
    x_offset = randint(0, height)
    y_offset = randint(0, width)
    l_img[y_offset:y_offset+s_img.shape[0], x_offset:x_offset+s_img.shape[1]] = s_img

    cv2.imwrite('/home/lab1/nitay/buildings/big_images/' + filename, l_img)
    print filename
