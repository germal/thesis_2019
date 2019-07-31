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
from sensor_msgs.msg import Image



class Move_BB8():

    def __init__(self, loop):
        print ("Constructor for class")
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 100)
        self.rate = rospy.Rate(10) # 10Hz freq
        self.loop = loop
        self.ctrl_c = False #  This is an indicator whether user has terminated the program yet or not/
        self.vel = Twist() # the variable with the type Twist
        self.vel.linear.x = self.loop


        self.random1 = 0
        self.random2 = 0
        self.global_joint_1 = 0
        self.global_joint_2 = 0

        rospy.on_shutdown(self.shutdownhook) # this gets triggered on shutdown

        self.bridge_object = CvBridge()
        self.camera_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.camera_callback, queue_size = 1)

        self.serv = rospy.Service('Activate_Motion', ActivateMotion  ,self.handle_service)
    def handle_service(self, req):
        resp = ActivateMotionResponse()
        resp.joint_angle_1 = self.global_joint_1
        resp.joint_angle_2 = self.global_joint_2
        return resp




    def camera_callback(self, data):

            jx1,jx2,jx3,jy1,jy2,jy3 = 0,0,0,0,0,0

            left_shoulder, left_elbow= 0,0

            try:
                cv_image = self.bridge_object.imgmsg_to_cv2(data,"bgr8")
                # converting to CV2
                #np_arr = np.fromstring(data.data, np.uint8)
                #cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            except CvBridgeError as e:
                print(e)

            height, width, channels = cv_image.shape
            descentre = 160
            rows_to_watch = 100
        #    crop_img = cv_image[(height)/2+descentre:(height)/2+(descentre+rows_to_watch)][1:width]

            # Convert RGB to HSV
    #        hsv =cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
            hsv =cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            lower_yellow = np.array([20,150,100])
            upper_yellow = np.array([50,255,255])

            lower_pink = np.array([190,100,100])
            upper_pink = np.array([240,255,255])

            lower_navy = np.array([130,150,80])
            upper_navy = np.array([170,255,200])


# threshold yellow only
            mask1 = cv2.inRange(hsv, lower_yellow, upper_yellow)
            mask2 = cv2.inRange(hsv, lower_pink, upper_pink)
            mask3 = cv2.inRange(hsv, lower_navy, upper_navy)


#YELLLOW

# calculate the centre of the blob image
            m1 = cv2.moments(mask1, False)
            try:
                cx1, cy1 = m1['m10']/m1['m00'], m1['m01']/m1['m00']

                # added for debugging
            except ZeroDivisionError:
                cy1, cx1 = height/2, width/2

            if (cy1 != height/2) and (cx1 != width/2):

                jx1, jy1 = cx1,cy1
                rospy.loginfo("first joint =   %f  and y  = %f", jx1, jy1)

    #  && with mask and original image
            #    res = cv2.bitwise_and(crop_img,crop_img, mask= mask)
                res1 = cv2.bitwise_and(cv_image,cv_image, mask= mask1)



    # show circles on the image
                cv2.circle(res1,(int(cx1), int(cy1)), 10,(0,0,255),-1)


                cv2.imshow("RES", res1)
                cv2.imshow("Image",cv_image)

                cv2.waitKey(1)


# PINK
            m2 = cv2.moments(mask2, False)
            try:
                cx2, cy2 = m2['m10']/m2['m00'], m2['m01']/m2['m00']
            except ZeroDivisionError:
                cy2, cx2 = height/2, width/2

            if (cy2 != height/2) and (cx2 != width/2):
                jx2, jy2 = cx2,cy2
                rospy.loginfo("second joint =   %f  and y  = %f", jx2, jy2)

    #  && with mask and original image
            #    res = cv2.bitwise_and(crop_img,crop_img, mask= mask)
                res2 = cv2.bitwise_and(cv_image,cv_image, mask= mask2)


    # show circles on the image
                cv2.circle(res2,(int(cx2), int(cy2)), 10,(0,0,255),-1)


                cv2.imshow("RES", res2)
                cv2.imshow("Image",cv_image)

                cv2.waitKey(1)


# BLUE

            m3 = cv2.moments(mask3, False)
            try:
                cx3, cy3 = m3['m10']/m3['m00'], m3['m01']/m3['m00']
            except ZeroDivisionError:
                cy3, cx3 = height/2, width/2

            if (cy3 != height/2) and (cx3 != width/2):

                jx3, jy3 = cx3,cy3
                rospy.loginfo("third joint =   %f  and y  = %f", jx3, jy3)

    #  && with mask and original image
            #    res = cv2.bitwise_and(crop_img,crop_img, mask= mask)
                res3 = cv2.bitwise_and(cv_image,cv_image, mask= mask3)


    # show circles on the image
                cv2.circle(res3,(int(cx3), int(cy3)), 10,(0,0,255),-1)


                cv2.imshow("RES", res3)
                cv2.imshow("Image",cv_image)

                cv2.waitKey(1)


            # If shoulder and elbow are picked up
            if jx1!=0 and jy1!=0 and jx2!=0 and jy2!=0:
    #            linethickness = 2
    #            cv2.line(cv_image, int(jx1, jy1), int(jx2, jy2), (0,255, 0),linethickness)
                left_shoulder = np.arctan((jy2-jy1)/(jx2-jx1))
                rospy.loginfo("left shoulder angle = %f", angle1)
                self.global_joint_1 = angle1


            # If elbow and wrist are picked up
            if jx2!=0 and jy2!=0 and jx3!=0 and jy3!=0:
                #            linethickness = 2
                #            cv2.line(cv_image, int(jx1, jy1), int(jx2, jy2), (0,255, 0),linethickness)
                left_elbow= left_shoulder + np.arctan((jy3-jy2)/(jx3-jx2))
                rospy.loginfo("joint angle 2 =   %f", angle2)
                self.global_joint_2 = angle2


            self.testGlobal = str(cx1);

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
    rospy.init_node('blob_detector', anonymous=True)
    obj = Move_BB8(0)
    try:
        obj.run_bb8()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
