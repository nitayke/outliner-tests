import cv2
import os
from constants import *
from random import randint

directory = "../2_removed_thin_lines/"

count = 0

for filename_int in range(1, FILES_COUNT + 1):
    count += 1
    filename = str(filename_int) + '.jpg'
    img = cv2.imread(directory + filename)

    height, width, _ = img.shape
    new_height, new_width = 0, 0

    i = 0

    # if i >= 5, the image is probably not in the correct ratio.
    while (not 100 < new_height < 300 or not 100 < new_width < 300) and i < 5:
        new_width = width / (randint(width//3, width) / 100)
        new_height = (height / (randint(height//3, height) / 100)) * (height/width)

        new_width, new_height = int(new_width), int(new_height)

        new_img = cv2.resize(img, (new_width, new_height), interpolation=cv2.INTER_NEAREST)
        i += 1

    cv2.imwrite('../3_after_resize/' + filename, new_img)
    print(filename, (new_width, new_height))

print('count: ', count)  # how many images are in the correct ratio (should be 510)