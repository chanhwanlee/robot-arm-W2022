#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ideasclinic.msg import class_info
import cv2
import numpy as np
import process
import tensorflow as tf
from tensorflow.keras.models import model_from_json


class client:
    def __init__(self):
        json_file = open("/home/kaeun/catkin_ws/src/ideasclinic/src/model.json", "r")
        loaded_model_json = json_file.read()
        json_file.close()
        self.loaded_model = model_from_json(loaded_model_json)

        self.loaded_model.load_weights("/home/kaeun/catkin_ws/src/ideasclinic/src/model.h5")
        print("Loaded model from disk")

        self.loaded_model.compile(loss='sparse_categorical_crossentropy', optimizer='adam', metrics=['accuracy']) 
        self.camera_rgb = rospy.Subscriber("class_detection/image_raw", Image, self.callback, queue_size=1)
        self.pub = rospy.Publisher("class_info", class_info, queue_size = 10)
        self.last_time = rospy.Time().now().to_sec()
        self.latest_img = None
        self.iter = 1
        
    def callback(self, img): 
        print("got image: %d" % self.iter)
        self.iter = self.iter + 1
        if self.iter % 30 != 0:
            return

        height = img.height
        width = img.width
        blank_image = np.zeros((height,width,3), np.uint8)

        row_size = width * 3

        for i in range(height):
            for j in range(width):
                for k in range(3):
                    idx = i*row_size + j*3 + k
                    blank_image[i, j, k] = img.data[idx]
        print("copied")
        #try:
        #     cv_image = CvBridge().imgmsg_to_cv2(img, "bgr8")
        #cv2.imshow("Classification Camera", blank_image)
        #cv2.waitKey(5)

        #except CvBridgeError as e:
        #    print(e)

        self.latest_img = blank_image

    def process(self):
        if self.latest_img is None:
            return
        cv_image = self.latest_img
        
        IMG_SIZE = 50
#        img2, contours, hierarchy = cv2.findContours(cv_image, cv2.CV_RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
 #       cnt = contours[0]
  #      M = cv2.moments(cnt)
   #     print(M)
        crop_img = cv_image[250:450, 250:450]
        cv2.imshow("cropped", crop_img)
        cv2.waitKey(5)
        img_array = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
        
                
       # img2, contours, hierarchy = cv2.findContours(img_array, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
       # cnt = contours[0]
       # M = cv2.moments(cnt)
       # print(M)
        #cnt = []
        #for i in range(len(contours)):
            #if cv2.contourArea(contours[i]) < 1000000:
                #cnt.append(contours[i]) 

        #print("area is", cnt)

        #pts_src = np.array([[250, 131], [340, 131], [340, 321], [250, 321]])
        #pts_dst = np.array([[0, 0], [299, 0], [299, 399], [0, 399]])
        
        #h, status = cv2.findHomography(pts_src, pts_dst)
        #im_out = cv2.warpPerspective(img_array, h, (img_array.shape[1], img_array.shape[0]))
        #cv2.imshow("Wraped Source Image", im_out)
        #cv2.waitKey(5)
        new_array = cv2.resize(img_array, (IMG_SIZE, IMG_SIZE))
        new_array = np.reshape(new_array, (-1, IMG_SIZE, IMG_SIZE, 1))
        #pts_src = np.array([[141, 131], [480, 159], [493, 630], [64, 601]])
        #pts_dst = np.array([[318, 256], [534, 372], [316, 670], [73, 473]])

        #h, status = cv2.findHomography(pts_src, pts_dst)
        #im_out = cv2.warpPerspective(cv_image, h, (cv_image.shape[1], cv_image.shape[0]))
       # cv2.imshow("Wraped Source Image", im_out)
        #new_array.reshape(1, IMG_SIZE, IMG_SIZE, 1)
        
       # model = tf.keras.models.load_model("CNN.model")
#        image = new_img
        '''json_file = open("/home/kaeun/catkin_ws/src/ideasclinic/src/model.json", "r")
        loaded_model_json = json_file.read()
        json_file.close()
        loaded_model = model_from_json(loaded_model_json)

        loaded_model.load_weights("/home/kaeun/catkin_ws/src/ideasclinic/src/model.h5")
        print("Loaded model from disk")

        loaded_model.compile(loss='sparse_categorical_crossentropy', optimizer='adam', metrics=['accuracy'])'''
        CATEGORIES=["cat", "dog", "spider", "squirrel"]
        prediction = self.loaded_model.predict_classes(new_array)
        #prediction = list(prediction[0])
        print(CATEGORIES[max(prediction)])
        print("prediction is", prediction)
        #model = keras.models.load_model("model.h5")
        #prediction = model.predict(cv_image)
        #prediction = list(prediction[0])
        #print(CATEGORIES[prediction.index(max(prediction))])

        msg = class_info()
        if prediction is not None:
            msg.symbol = prediction

            self.pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('img_init')
    c = client()
#    rospy.spin()
    while not rospy.is_shutdown():
        rospy.sleep(1)
#        time in seconds
        c.process()

