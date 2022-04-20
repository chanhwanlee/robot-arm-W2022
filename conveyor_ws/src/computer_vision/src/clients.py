#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ideasclinic.msg import box_info
import cv2
import numpy as np
import process
#from process import box, preprocess_image, corner, contour, test, draw
#pub = None

class client:
    def __init__(self):
        self.camera_rgb = rospy.Subscriber("/edge_detection/image_raw", Image, self.callback)
        self.pub = rospy.Publisher("box_info", box_info, queue_size = 10)
        self.last_time = rospy.Time().now().to_sec()
        self.latest_img = None

    def callback(self, img):
#        t = rospy.Time().now().to_sec()
#        if t - self.last_time < 5:
#            print("not yet")
#            return

#        self.last_time = t    
        print("got image")
        try:
             cv_image = CvBridge().imgmsg_to_cv2(img, "bgr8")
             cv2.imshow("Edge Detection Camera", cv_image)
             cv2.waitKey(25)
         
        except CvBridgeError as e:
            print(e)

#        t = rospy.Time().now().to_sec()
#        if t - self.last_time < 5:
#            print("not yet")
#            return

#        self.last_time = t

        self.latest_img = cv_image
    
    def process(self):
        if self.latest_img is None:
            return
        cv_image = self.latest_img

#        img_array = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        pts_src = np.array([[220, 231], [320, 231], [320, 371], [220, 371]])
        pts_dst = np.array([[0, 0], [299, 0], [299, 399], [0, 399]])

        h, status = cv2.findHomography(pts_src, pts_dst)
        im_out = cv2.warpPerspective(cv_image, h, (cv_image.shape[1], cv_image.shape[0]))
        cv2.imshow("Wraped Source Image", im_out)
        cv2.waitKey(5)

        threshold_image = process.thresh_image(im_out)
        cnts_sort, cnt_is_card = process.contour(threshold_image)

        kaeun = None
        if len(cnts_sort) != 0:
            cards = []
            k = 0
            for i in range(len(cnts_sort)):
                if(cnt_is_card[i] == 1):
                    cards.append(process.find_information(cnts_sort[i], cv_image))
                    cv_image = process.draw(cv_image, cards[k])
                    #im_out = process.draw(im_out, cards[k])
                    kaeun = cards[k]
                    k = k + 1
            if (len(cards) != 0):
                temp_cnts = []
                for i in range(len(cards)):
                    temp_cnts.append(cards[i].contour) 
                cv2.drawContours(cv_image, temp_cnts, -1, (255, 0, 0), 2)
                #cv2.drawContours(im_out, temp_cnts, -1, (255, 0, 0), 2)

        cv2.imshow("threshold", cv_image)
        #cv2.imshow("threshold", im_out)

#        t = rospy.Time().now().to_sec()
#        if t - self.last_time < 5:
#            print("not yet")
#            return

#        self.last_time = t
        
       # cv2.waitKey(25)
       # kaeun = cards 
        msg = box_info()
        if kaeun is not None:
            msg.x = kaeun.center[0]
            msg.y = kaeun.center[1]
            msg.w = kaeun.width
            msg.h = kaeun.height
            msg.angle = kaeun.angle
    
            self.pub.publish(msg)
#    corners = corner(cv_image, threshold)
#
#    cv2.imshow("Corner", corners)
#    cv2.waitKey(25)

#    find_contour = contour(threshold)
#    cv2.imshow("contour",find_contour)
#    cv2.waitKey(25)

#    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
#    edged = cv2.Canny(gray, 30, 200)
#    img, contours, hierarchy = cv2.findContours(edged, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
#    kaeun_1 = process.test(cv_image, kaeun)
#    image = process.draw(cv_image, kaeun_1)
#    cv2.drawContours(cv_image, find_test, -1, (0, 255, 0), 3)
#    cv2.imshow('test', image)
#    cv2.waitKey(25)

#    def listener():
#        rospy.init_node('img_init')
#        print("Starting client")

#        camera_rgb = rospy.Subscriber("/edge_detection_rgb/rgb/image", Image, callback)

#        pub = rospy.Publisher("box_info", box_info, queue_size = 10)

#        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('img_init')
    c = client()
#    rospy.spin()
    while not rospy.is_shutdown():
        rospy.sleep(1)
#        time in seconds
        c.process()

