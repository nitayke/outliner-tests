# step 1

import cv2
import numpy as np
import os

i = 1

for filename in os.listdir("../original"):
    img = cv2.imread('../original/' + filename)
    kernel = np.ones((5,5),np.uint8)
    opening = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)

    cv2.imwrite('../removed_thin_lines/' + str(i) + '.jpg', opening)
    print(i)
    i += 1