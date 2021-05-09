import cv2
import os
from constants import *
from random import randint
import numpy as np

directory = "../2_removed_thin_lines/"

def find_rect(img):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    indexes = np.argwhere(gray < 10)

    # [y1, x1], [y2, x2]
    return np.min(indexes, axis=0), np.max(indexes, axis=0)


for filename_int in range(1, FILES_COUNT + 1):
    filename = str(filename_int) + '.jpg'
    image = cv2.imread(directory + filename)
    print(filename)
    
    [y1, x1], [y2, x2] = find_rect(image)

    cropped = image[y1:y2, x1:x2]
    
    cv2.imwrite('../cropped/'+filename, cropped)