import numpy as np
import cv2
import time
import math
import sympy
from sympy.solvers import solve
from sympy import Symbol

BKG_THRESH = 20
CARD_MAX_AREA = 20000
CARD_MIN_AREA = 20

class box:
    def __init__(self):
        self.contour = []
        self.width, self.height = 0, 0
        self.corner_pts = []
        self.center = []
        self.angle = 0

def thresh_image(image):
    #this function thresholds the image so that it is easier to detect the edges
    testbox = box()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    img_w, img_h = np.shape(image)[:2]
    bkg_level = gray[int(img_h/100)][int(img_w/2)]
    thresh_level = bkg_level + BKG_THRESH
    #threh_level can also be determined by trial&error
    retval, thresh = cv2.threshold(blur,thresh_level,255,cv2.THRESH_BINARY)

    return thresh

def corner(ori_img, thresh_img):
    #this finds the corners of an image but is very inaccurate
    corners = cv2.goodFeaturesToTrack(thresh_img, 4, 0.01, 20)
    corners = np.int0(corners)
    print(corners)
    for corner in corners:
        x, y = corner.ravel()
        cv2.circle(ori_img, (x, y), 3, 255, -1)

    return ori_img

def contour(thresh_image):
    #this function finds the contour of an object
    #it filters out the "card" depending on the size and number of corners
    #also, it checks if it has any "parent contour"

    img2, contours, hierarchy = cv2.findContours(thresh_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    index_sort = sorted(range(len(contours)), key=lambda i : cv2.contourArea(contours[i]),reverse=True)
    if len(contours) == 0:
        return [], []
    cnts_sort = []
    hier_sort = []
    cnt_is_card = np.zeros(len(contours), dtype=int)

    for i in index_sort:
        cnts_sort.append(contours[i])
        hier_sort.append(hierarchy[0][i])
    for i in range(len(cnts_sort)):
        size = cv2.contourArea(cnts_sort[i])
        peri = cv2.arcLength(cnts_sort[i], True)
        approx = cv2.approxPolyDP(cnts_sort[i], 0.01*peri, True)

        if ((size < CARD_MAX_AREA) and (size > CARD_MIN_AREA) and (hier_sort[i][3] == -1) and (len(approx) == 4)):
                cnt_is_card[i] = 1
    return cnts_sort, cnt_is_card

def find_information(contour, image):
    #this functions actually gets the information of the box
    kaeun = box()
    kaeun.contour = contour
    peri = cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, 0.01*peri, True)
    pts = np.float32(approx)
    kaeun.corner_pts = pts

#   The below method also gets the width and height, but it seems off
#    x, y, w, h = cv2.boundingRect(contour)
#    kaeun.width, kaeun.height = w, h

#   The below method is an alternative which performs a pythagoras theorem
    temp_length1, temp_length2 = 0, 0
    temp_x1 = kaeun.corner_pts[0][0][0]
    temp_x2 = kaeun.corner_pts[3][0][0]
    temp_x3 = kaeun.corner_pts[2][0][0]
    temp_y1 = kaeun.corner_pts[0][0][1]
    temp_y2 = kaeun.corner_pts[3][0][1]
    temp_y3 = kaeun.corner_pts[2][0][1]
    temp_length1 = math.sqrt(abs(temp_x1 - temp_x2)**2 + abs(temp_y1 - temp_y2)**2)
    temp_length2 = math.sqrt(abs(temp_x3 - temp_x2)**2 + abs(temp_y3 - temp_y2)**2)
    if(temp_length1 > temp_length2):
        kaeun.height = temp_length1
        kaeun.width = temp_length2
    else:
        kaeun.height = temp_length2
        kaeun.width = temp_length1

    average = np.sum(pts, axis=0)/len(pts)
    cent_x = int(average[0][0])
    cent_y = int(average[0][1])
    kaeun.center = [cent_x, cent_y]
    print("edge", kaeun.corner_pts)
    print("width: ", kaeun.width)
    print("height: ", kaeun.height)
    
#   The below method finds the angle of the box (angle between the long axis of the box and the x axis)
    
    middle_x = (temp_x1 + temp_x2)/2
    middle_y = (temp_y1 + temp_y2)/2
    middle = [middle_x, middle_y]

    dist_cent_mid = math.sqrt(abs(middle[0] - kaeun.center[0])**2 + abs(middle[1] - kaeun.center[1])**2)

    ratio_a = kaeun.center[1]
    ratio_b = middle[1]

    x = Symbol('x')
    hypo_array = solve((ratio_b * x) - (ratio_a * (x - dist_cent_mid)), x)
    hypo = hypo_array[0]
    oppo = kaeun.center[1]
    print("oppo/hypo:", oppo/hypo)
    print("hypo:", hypo)
    print("oppo:", oppo)
    temp_angle_radian = math.asin(oppo/hypo)
    temp_angle_1 = math.degrees(temp_angle_radian)
    temp_angle_2 = 180 - temp_angle_1
    
    if(temp_length1 < temp_length2):
        kaeun.angle = temp_angle_1
    else:
        kaeun.angle = temp_angle_2

    print("angle: ", kaeun.angle)
    
    
    return kaeun

def draw(image, kaeun):
#   This function displays the found information of the box (the center and the corners) in the camera
    x = kaeun.center[0]
    y = kaeun.center[1]
    cv2.circle(image, (x, y), 5, (255, 0, 0), -1)
    print(x, y)

    for i in range(len(kaeun.corner_pts)):
        x = kaeun.corner_pts[i][0][0]
        y = kaeun.corner_pts[i][0][1]
        cv2.circle(image, (x, y), 5, (0, 255, 0), -1)
#        cv2.circle(image, (0, 0), 20, (0, 255, 255), -1)
    return image

