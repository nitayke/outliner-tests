import cv2
import os
from constants import *
from random import randint
import numpy as np

directory = "../2_removed_thin_lines/"

count = 0

def crop_minAreaRect(img, rect):
    angle = rect[2]
    height, width = img.shape[0], img.shape[1]
    height, width, _ = img.shape
    M = cv2.getRotationMatrix2D((width/2,height/2),angle,1)
    img_rot = cv2.warpAffine(img,M,(width,height))
    box = cv2.boxPoints(rect)
    pts = np.int0(cv2.transform(np.array([box]), M))[0]

    pts[pts < 0] = 0
    img_crop = img_rot[pts[1][1]:pts[0][1], 
                       pts[1][0]:pts[2][0]]
    return img_crop

for filename_int in range(1, FILES_COUNT + 1):
    filename = str(filename_int) + '.jpg'
    image = cv2.imread(directory + filename)

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]

    cnts = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    im = np.zeros((IMAGE_SIZE, IMAGE_SIZE, 3), np.uint8) # blank image
    if len(cnts) > 1:
        tmp = cnts[0]
        for i in cnts[1:]:
            tmp = np.append(tmp, i, axis=0)
        cnts = [tmp]
    rect = cv2.minAreaRect(cnts[0])

    cropped = crop_minAreaRect(image, rect)
    
    height, width, _ = cropped.shape
    new_height, new_width = 0, 0

    i = 0

    # if i >= 10, the image is probably not in the correct ratio.
    while (not 100 < new_height < 300 or not 100 < new_width < 300) and i < 10:
        new_width = width / (randint(width//3, width) / 100)
        new_height = new_width * (height/width)

        new_width, new_height = int(new_width), int(new_height)
        print('Trying again:', filename, new_height, new_width)
        new_image = cv2.resize(cropped, (new_width, new_height), interpolation=cv2.INTER_NEAREST)
        i += 1
    
    print(filename, new_height, new_width, '\n')

    if i < 10:
        count += 1
        cv2.imwrite('../3_after_resize/' + filename, new_image)

print('count: ', count)  # how many images are in the correct ratio