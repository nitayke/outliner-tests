import cv2
import os

directory = "/home/lab1/nitay/buildings/dataset/"

for filename in os.listdir(directory):
    img = cv2.imread(directory + filename)

    img = cv2.resize(img, (int(img.shape[1]/10), int(img.shape[0]/10)), interpolation=cv2.INTER_NEAREST)

    cv2.imwrite('/home/lab1/nitay/buildings/small_dataset/' + filename, img)
    print filename