#!/usr/bin/env python

import rospy
import clients
from ideasclinic.msg import box_info

class talk_robot:
    def __init__(self):
        self.subscriber_box = rospy.Subscriber("box_info", box_info, self.callback)

        self.subscriber_class = rospy.Subscriber("class_info", class_info, self.callback_class)

        self.robot = serial.Serial('COM1', 9600, timeout=0.1)


    def callback(self, info):
        #print(info)
        #print(info.x)
        robot.write(str(info.x), str(info.y), str(info.w), str(info.h), str(info.angle))

    def callback_class(self, info):
       robot.write(str(info.symbol))


if __name__ == '__main__':
    t = talk_robot()
    ros.init_node('info_init')
    rospy.spin()
