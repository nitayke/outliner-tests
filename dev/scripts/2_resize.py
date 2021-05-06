import cv2
import os
from constants import *
from random import randint
import numpy as np

directory = "../2_removed_thin_lines/"

count = 0

for filename_int in range(1, FILES_COUNT + 1):
    filename = str(filename_int) + '.jpg'
    original = cv2.imread(directory + filename)
    gray = cv2.cvtColor(original, cv2.COLOR_BGR2GRAY)
    thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]

    cnts = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    im = np.zeros((original.shape[1], original.shape[0], 3), np.uint8) # swap them?

    if len(cnts) > 1:
        tmp = cnts[0]
        for i in cnts[1:]:
            tmp = np.append(tmp, i, axis=0)
        cnts = [tmp]
    cv2.drawContours(im, cnts, 0, (0,255,0), 3)

    ((x, y), (width, height), angle) = cv2.minAreaRect(cnts[0])

    # box = cv2.boxPoints(((x, y), (width, height), angle))
    # box = np.int0(box)
    # cv2.drawContours(original, [box], 0, (0, 0, 255), 3)

    y_offset = int(y-0.5*height + 10)
    x_offset = int(x-0.5*width + 10)

    print(y_offset, x_offset)
    
    cropped = original[y_offset:int(y_offset+height), x_offset:int(x_offset+width)]
    
    height, width, _ = cropped.shape
    new_height, new_width = 0, 0

    i = 0

    # if i >= 10, the image is probably not in the correct ratio.
    while (not 100 < new_height < 300 or not 100 < new_width < 300) and i < 10:
        new_width = width / (randint(width//3, width) / 100)
        new_height = new_width * (height/width)

        new_width, new_height = int(new_width), int(new_height)
        new_image = cv2.resize(cropped, (new_width, new_height), interpolation=cv2.INTER_NEAREST)
        i += 1
    

    if i < 10:
        count += 1
        cv2.imwrite('../3_after_resize/' + filename, new_image)

print('count: ', count)  # how many images are in the correct ratio