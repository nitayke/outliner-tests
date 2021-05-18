import sys
import os
import cv2
import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from map_roi_aligner_msgs.srv import MapBuildingOutline
from geometry_msgs.msg import Pose
from random import randint
from math import radians, degrees, cos, sin, pi
import glob
from yaml import load

big_images = {}
ground_truth = {}
errors = {}
done = False

with open('./config.yaml') as file:
    parameters = load(file)

IMAGE_SIZE = parameters['IMAGE_SIZE']
FILES_COUNT = parameters['FILES_COUNT']
MIN_BUILDING_SIZE = parameters['MIN_BUILDING_SIZE']
MAX_BUILDING_SIZE = parameters['MAX_BUILDING_SIZE']

fields = ['x', 'y', 'width', 'height', 'angle']
all_count = 0


def get_filename(filename_int):
    return str(filename_int % FILES_COUNT + 1) + '.jpg'


def rotate_image(image, angleInDegrees):
    h, w = image.shape[:2]
    img_c = (w / 2, h / 2)

    rot = cv2.getRotationMatrix2D(img_c, angleInDegrees, 1)

    rad = radians(angleInDegrees)
    sin1 = sin(rad)
    cos1 = cos(rad)
    b_w = int((h * abs(sin1)) + (w * abs(cos1)))
    b_h = int((h * abs(cos1)) + (w * abs(sin1)))

    rot[0, 2] += ((b_w / 2) - img_c[0])
    rot[1, 2] += ((b_h / 2) - img_c[1])

    outImg = cv2.warpAffine(image, rot, (b_w, b_h), flags=cv2.INTER_LINEAR, borderValue=(255,255,255)) # INTER_LINEAR / INTER_NEAREST
    return outImg


def resize_image(image):
    height, width, _ = image.shape
    if not (MIN_BUILDING_SIZE/MAX_BUILDING_SIZE < width/height < MAX_BUILDING_SIZE/MIN_BUILDING_SIZE):
        print('Wrong image ratio!')
        return
    if height > width:
        new_width = randint(MIN_BUILDING_SIZE, int(MAX_BUILDING_SIZE*(width/height)))
        new_height = int(new_width * (height/width))
    else:
        new_height = randint(MIN_BUILDING_SIZE, int(MAX_BUILDING_SIZE*(height/width)))
        new_width = int(new_height * (width/height))
    return cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_NEAREST)


def find_rect(image, filename_int):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]

    cnts = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    im = np.zeros((IMAGE_SIZE, IMAGE_SIZE, 3), np.uint8)
    if len(cnts) > 1:
        tmp = cnts[0]
        for i in cnts[1:]:
            tmp = np.append(tmp, i, axis=0)
        cnts = [tmp]
    cv2.drawContours(im, cnts, 0, (0,255,0), 3)

    ((x, y), (width, height), angle) = cv2.minAreaRect(cnts[0])
    (x, y, width, height, angle) = (round(x/10, 3), round(y/10, 3), round(width/10, 3), round(height/10, 3), round(angle, 3))
    ground_truth[filename_int] = {'x': x, 'y': y, 'width': width, 'height': height, 'angle': radians(angle)}


def get_ogrid(image, filename):
    m = MapMetaData()
    m.resolution = 0.1
    m.width = m.height = IMAGE_SIZE
    m.origin = Pose()
    m.origin.position.x = m.origin.position.y = 0

    ogrid = OccupancyGrid()
    ogrid.header.frame_id = filename
    ogrid.info = m

    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ogrid.data = image.ravel().tolist()
    for i in range(len(ogrid.data)):
        ogrid.data[i] = 0 if 150 <= ogrid.data[i] <= 255 else 100
    
    return ogrid


def compare_results(filename_int, foldername):
    im1 = cv2.imread('../templates/gt.jpg')
    im2 = cv2.imread('../templates/result.jpg')

    transparent1 = big_images[filename_int].copy()
    transparent2 = big_images[filename_int].copy()

    gt = list(ground_truth[filename_int].values())

    box1 = cv2.boxPoints(((gt[0]*10, gt[1]*10), (gt[2]*10, gt[3]*10), degrees(gt[4])))
    box1 = np.int0(box1)
    cv2.drawContours(transparent1,[box1],0,(0x77,0x77,0xff),2)

    error = errors[filename_int]

    box2 = cv2.boxPoints(((error[1][0]*10, error[1][1]*10), (error[1][2]*10, error[1][3]*10), degrees(error[1][4])))
    box2 = np.int0(box2)
    cv2.drawContours(transparent2,[box2],0,(0xff, 0xa7, 0x4f),2)

    im1[50:, :] = transparent1
    im2[50:, :] = transparent2

    hori = np.concatenate((im1, im2), axis=1)

    cv2.imwrite(foldername + get_filename(filename_int), hori)


def get_final_results():
    global all_count
    count = 0
    std = np.std([i[0] for i in errors.values()], axis=0).tolist()
    std = [round(i, 3) for i in std]

    for filename_int, error in errors.items():
        print('Processing', filename_int)
        for i in range(5):
            if i == 0:
                compare_results(filename_int, '../compared/')
            if abs(error[0][i]) > 2*std[i]:
                count += 1
                compare_results(filename_int, '../big_errors/')
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
    print(count, 'of', all_count)


def delete_images(path):
    files = glob.glob(path)
    for f in files:
        os.remove(f)


def service_client(map):
    rospy.wait_for_service('map_roi/map_building_outline')
    try:
        build_outline = rospy.ServiceProxy('map_roi/map_building_outline', MapBuildingOutline)
        resp = build_outline(map)
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def add_result(data, filename_int):
    global ground_truth
    global all_count

    all_count += 1

    print('Adding', filename_int)

    gt = [ground_truth[filename_int]['x'], ground_truth[filename_int]['y'], ground_truth[filename_int]['width'],
    ground_truth[filename_int]['height'], ground_truth[filename_int]['angle']]

    result = [data.optimizedRect.centerX, data.optimizedRect.centerY, data.optimizedRect.width, data.optimizedRect.height, data.optimizedRect.theta]
    
    # 90 degrees rotated
    if -0.1 < abs(gt[4] - result[4]) - pi/2 < 0.1:
        gt[2], gt[3] = gt[3], gt[2]
        gt[4] -= pi/2

    single_errors = [gt[i] - result[i] for i in range(5)]
    errors[filename_int] = [single_errors, result]


def main():
    global done
    if len(sys.argv) != 2 or not sys.argv[1].isdigit():
        print("Usage: python3 %s <images_count>" % sys.argv[0])
        return

    rospy.init_node('outliner_test')

    delete_images('../compared/*')
    delete_images('../big_errors/*')

    if int(sys.argv[1]) == 0: # Just deleting files
        print('Cleaned successfully')
        return

    rospy.sleep(1.)

    directory = "../dataset/"

    for filename_int in range(int(sys.argv[1])):
        s_img = cv2.imread(directory + get_filename(filename_int))
        s_img = resize_image(s_img)

        l_img = np.zeros((IMAGE_SIZE, IMAGE_SIZE, 3), np.uint8)
        l_img.fill(255)
        
        # rotate
        while True:
            angle = randint(-45, 45)
            rotated = rotate_image(s_img, angle)
            height, width, _ = rotated.shape
            if height < IMAGE_SIZE and width < IMAGE_SIZE:
                break

        x_offset = randint(0, IMAGE_SIZE-width)
        y_offset = randint(0, IMAGE_SIZE-height)

        # place in constant big image
        l_img[y_offset:y_offset+height, x_offset:x_offset+width] = rotated
        big_images[filename_int] = l_img

        find_rect(l_img, filename_int)

        ogrid = get_ogrid(l_img, get_filename(filename_int))
        result = service_client(ogrid)
        if result.solutionUsable:
            add_result(result, filename_int)

    get_final_results()
    

if __name__ == '__main__':
    main()