import cv2
import os

directory = "./big_images/"
f = open('./coordinates', 'w+')
for filename in os.listdir(directory):
    l_img = cv2.imread(directory + filename)
    gray = cv2.cvtColor(l_img, cv2.COLOR_BGR2GRAY)
    thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]

    cnts = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    ((x, y), (width, height), angle) = cv2.minAreaRect(cnts[0])
    (x, y, width, height, angle) = (round(x, 2), round(y, 2), round(width, 2), round(height, 2), round(angle, 2))
    d = {'x': x, 'y': y, 'width': width, 'height': height, 'angle': angle}
    f.write(filename + ': ' + str(d) + '\n')
    print filename

f.close()