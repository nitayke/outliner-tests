import json
import cv2
import numpy as np
from math import degrees
from constants import *

count = 0

file = open('../results.json', 'r')
errors = json.loads(file.read())
file.close()

file = open('../gt.json', 'r')
gt_dict = json.loads(file.read())
file.close()

std = np.std([i[0] for i in errors.values()], axis=0).tolist()
std = [round(i, 3) for i in std]

for filename, error in errors.items():
    print(filename)
    for i in range(5):
        if abs(error[0][i]) > 2*std[i]:
            count += 1

            im1 = cv2.imread('../templates/gt.jpg')
            im2 = cv2.imread('../templates/result.jpg')

            transparent1 = cv2.imread('../4_final/' + filename)
            transparent2 = cv2.imread('../4_final/' + filename)

            gt = list(gt_dict[filename].values())

            box1 = cv2.boxPoints(((gt[0]*10, gt[1]*10), (gt[2]*10, gt[3]*10), degrees(gt[4])))
            box1 = np.int0(box1)
            cv2.drawContours(transparent1,[box1],0,(0x77,0x77,0xff),2)

            box2 = cv2.boxPoints(((error[1][0]*10, error[1][1]*10), (error[1][2]*10, error[1][3]*10), degrees(error[1][4])))
            box2 = np.int0(box2)
            cv2.drawContours(transparent2,[box2],0,(0xff, 0xa7, 0x4f),2)

            im1[50:, :] = transparent1
            im2[50:, :] = transparent2

            hori = np.concatenate((im1, im2), axis=1)

            cv2.imwrite('../6_debugging/' + filename, hori)
            break

print('------ Average error: ----------')
for i in range(4):
    print(fields[i], ':', round(sum([j[0][i] for j in errors.values()])/FILES_COUNT, 3), '[m]')
print('angle:', round(sum([i[0][4] for i in errors.values()])/FILES_COUNT, 3), '[°]')

print('------ Standard Deviation: -----')
for i in range(4):
    print(fields[i], ':', std[i], '[m]')
print(fields[4], ':', std[4], '[°]')

print('------ Number of big errors: ---')
print(count)