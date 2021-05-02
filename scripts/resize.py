# step 2

import cv2
import os

directory = "../removed_thin_lines/"

for filename in os.listdir(directory):
    img = cv2.imread(directory + filename)

    img = cv2.resize(img, (int(img.shape[1]/10), int(img.shape[0]/10)), interpolation=cv2.INTER_NEAREST)

    cv2.imwrite('../after_resize/' + filename, img)
    print(filename)