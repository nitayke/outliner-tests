import cv2
import numpy as np
import os

for filename in os.listdir("/home/lab1/Downloads/ROBIN/"):
    img = cv2.imread('/home/lab1/Downloads/ROBIN/' + filename)
    kernel = np.ones((5,5),np.uint8)
    opening = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)

    cv2.imwrite('/home/lab1/Downloads/NEW_ROBIN/' + filename, opening)