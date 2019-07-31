#! /usr/bin/env python


import rospy
import sys
import roslib

import cv2

import numpy as np

from move_bb8_pkg.srv import *


from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry


class Move_BB8():

    def __init__(self, loop):
        print ("Constructor for class")
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 100)

        self.rate = rospy.Rate(10) # 10Hz freq
        self.loop = loop
        self.ctrl_c = False #  This is an indicator whether user has terminated the program yet or not/
        self.vel = Twist() # the variable with the type Twist
        self.vel.linear.x = self.loop


        self.testGlobal = 0

        rospy.on_shutdown(self.shutdownhook) # this gets triggered on shutdown

        self.bridge_object = CvBridge()
        self.camera_sub = rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.camera_callback, queue_size = 1)



        self.serv = rospy.Service('Activate_Motion', ActivateMotion  ,self.handle_service)
    def handle_service(self, req):
        return self.testGlobal

    def camera_callback(self, data):




            try:
                #cv_image = self.bridge_object.imgmsg_to_cv2(data)

                # converting to CV2
                np_arr = np.fromstring(data.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            except CvBridgeError as e:
                print(e)

            height, width, channels = cv_image.shape
            descentre = 160
            rows_to_watch = 100
        #    crop_img = cv_image[(height)/2+descentre:(height)/2+(descentre+rows_to_watch)][1:width]

            # Convert RGB to HSV
    #        hsv =cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
            hsv =cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            lower_yellow = np.array([25,100,100])
            upper_yellow = np.array([50,255,255])



# threshold yellow only
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)



# calculate the centre of the blob image
            m = cv2.moments(mask, False)
            try:
                cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
            except ZeroDivisionError:
                cy, cx = height/2, width/2





#  && with mask and original image
        #    res = cv2.bitwise_and(crop_img,crop_img, mask= mask)
            res = cv2.bitwise_and(cv_image,cv_image, mask= mask)


# show circles on the image
            cv2.circle(res,(int(cx), int(cy)), 10,(0,0,255),-1)


            cv2.imshow("RES", res)
            cv2.imshow("Image",cv_image)

            cv2.waitKey(1)

            adjust_x = cx - 300
            self.vel.linear.x = 0.1
            self.vel.angular.z = adjust_x / 1000

            if self.vel.angular.z > 0.05 or self.vel.angular.z < -0.05:
                self.vel.linear.x = 0.01

            rospy.loginfo("blob location =   %f  and y  = %f", cx, cy )
        #    self.move_bb8(); # move the bot with updated Twist

            self.testGlobal = str(cx);




    def run_bb8(self):

            while not self.ctrl_c:
                connections = self.pub.get_num_connections()
                if connections > 0:
                    self.pub.publish(self.vel)
                    rospy.loginfo("Cmd Published")
                    break
                else:
                    self.rate.sleep()



    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.vel.linear.x = 0
        self.vel.angular.z = 0
    #    self.move_bb8(); # move the bot with updated Twist
        self.ctrl_c = True

    def move_bb8(self):

        rospy.loginfo("Moving BB8!")
        self.run_bb8()


if __name__ == '__main__':
    rospy.init_node('bb8_move', anonymous=True)
    obj = Move_BB8(0)
    try:
        obj.run_bb8()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
