import cv2
import os
import numpy as np

directory = "./big_images/"
f = open('./coordinates', 'w+')
for filename in os.listdir(directory):
    l_img = cv2.imread(directory + filename)
    gray = cv2.cvtColor(l_img, cv2.COLOR_BGR2GRAY)
    thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]

    cnts = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    print len(cnts)
    im = np.zeros((250, 250, 3), np.uint8) # blank image
    cv2.drawContours(im, cnts, -1, (0,255,0), 3)

    ((x, y), (width, height), angle) = cv2.minAreaRect(cnts)
    (x, y, width, height, angle) = (round(x, 2), round(y, 2), round(width, 2), round(height, 2), round(angle, 2))
    d = {'x': x, 'y': y, 'width': width, 'height': height, 'angle': angle}
    f.write(filename + ': ' + str(d) + '\n')
    print filename
    box = np.int0(cv2.boxPoints(((x, y), (width, height), angle) ))
    cv2.drawContours(l_img, [box], 0, (36,255,12), 3)
    # cv2.imshow('image ' + filename, l_img)
    # cv2.waitKey()

    numpy_horizontal = np.hstack((im, l_img))
    numpy_horizontal_concat = np.concatenate((im, l_img), axis=1)
    cv2.imshow(filename, numpy_horizontal_concat)
    cv2.waitKey()

f.close()