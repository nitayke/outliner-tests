import cv2
import numpy as np
import os
from constants import *

i = 1

for filename in os.listdir("../1_original"):
    img = cv2.imread('../1_original/' + filename)
    kernel = np.ones((5,5),np.uint8)
    opening = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)

    cv2.imwrite('../2_removed_thin_lines/' + str(i) + '.jpg', opening)
    print(i)
    i += 1