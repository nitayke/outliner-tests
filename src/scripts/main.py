import os
import cv2
import numpy as np
from random import randint
from math import radians, cos, sin
import yaml
from constants import *
import sys

big_images = {}

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

    outImg = cv2.warpAffine(image, rot, (b_w, b_h), flags=cv2.INTER_NEAREST, borderValue=(255,255,255)) # / INTER_LINEAR
    return outImg


def resize_image(image):
    height, width, _ = image.shape
    if not (MIN_BUILDING_SIZE/MAX_BUILDING_SIZE < width/height < MAX_BUILDING_SIZE/MIN_BUILDING_SIZE):
        print('Wrong image ratio!')
        return
    if height > width:
        new_width = randint(MIN_BUILDING_SIZE, MAX_BUILDING_SIZE*(width/height))
        new_height = new_width * (height/width)
    else:
        new_height = randint(MIN_BUILDING_SIZE, MAX_BUILDING_SIZE*(height/width))
        new_width = new_height * (width/height)

    return cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_NEAREST)
    

def main():
    if len(argv) != 2:
        print("Usage: python3 main.py <images_count>")
        exit(0)
    directory = "../dataset/"

    for filename_int in range(1, int(sys.argv[1])):
        filename = str(filename_int % FILES_COUNT) + '.jpg'
        s_img = cv2.imread(directory + filename)
        try:
            print(s_img.shape[0], s_img.shape[1])
        except AttributeError:
            continue

        l_img = np.zeros((IMAGE_SIZE, IMAGE_SIZE, 3), np.uint8)
        l_img.fill(255)
        
        # rotate
        while True:
            angle = randint(-45, 45)
            rotated = rotate_image(s_img, angle)
            height, width, _ = rotated.shape
            if height < IMAGE_SIZE and width < IMAGE_SIZE:
                break

        x_offset = randint(0, IMAGE_SIZE-width)
        y_offset = randint(0, IMAGE_SIZE-height)

        # place in constant big image
        l_img[y_offset:y_offset+height, x_offset:x_offset+width] = rotated

        big_images[filename] = l_img
        
        print(filename)


if __name__ == '__main__':
    main()