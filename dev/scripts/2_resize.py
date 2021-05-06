import cv2
import os
from constants import *

directory = "../2_removed_thin_lines/"

for filename_int in range(1, FILES_COUNT + 1):
    filename = str(filename_int) + '.jpg'
    img = cv2.imread(directory + filename)

    img = cv2.resize(img, (int(img.shape[1]/7), int(img.shape[0]/7)), interpolation=cv2.INTER_NEAREST)

    cv2.imwrite('../3_after_resize/' + filename, img)
    print(filename)