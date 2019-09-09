#! /usr/bin/env python


# This program will just publish the linear and angular velocity in a way that
# it will make the robot move in square until the node is stopped.

import rospy
import sys
import roslib

import cv2

import numpy as np



from geometry_msgs.msg import Twist


from nav_msgs.msg import Odometry


class Move_BB8():

    def __init__(self, loop):
        print ("Constructor for class")
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

        self.start_time = rospy.Time()
        self.duration = rospy.Time()

        self.turn_flag = 0

        self.count = 0


        self.rate = rospy.Rate(10) # 10Hz freq
        self.loop = loop
        self.ctrl_c = False #  This is an indicator whether user has terminated the program yet or not/
        self.vel = Twist() # the variable with the type Twist
        self.vel.linear.x = self.loop


        self.testGlobal = 0

        rospy.on_shutdown(self.shutdownhook) # this gets triggered on shutdown



    def run_bb8(self):

            while not self.ctrl_c:
                connections = self.pub.get_num_connections()
                if connections > 0:



                        # if it has done three loops stop moving
                    if (self.count > 14):  # it will turn N times : count = 8*N - 2
                        self.vel.linear.x = 0
                        self.vel.angular.z = 0
                        self.pub.publish(self.vel)
                        rospy.loginfo("Done looping")
                        rospy.sleep(5)
                        self.turn_flag = 5

                        # go forward for about 1 metre
                    if (self.turn_flag ==0):
                        self.vel.angular.z = 0
                        self.vel.linear.x = 0.24
                        self.count = self.count + 1
                        self.pub.publish(self.vel)
                        rospy.loginfo("Cmd Published")
                        rospy.sleep(4)

                        self.turn_flag = 1

                    # turn left 90 degrees
                    if (self.turn_flag == 1):
                        self.vel.linear.x = 0
                        self.vel.angular.z = 0.3135
                        self.count = self.count + 1
                        self.pub.publish(self.vel)
                        rospy.loginfo("Cmd Published")
                        rospy.sleep(5)
                        self.turn_flag = 0












    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        self.pub.publish(self.vel)
    #    self.move_bb8(); # move the bot with updated Twist
        self.ctrl_c = True

    def move_bb8(self):

        rospy.loginfo("Moving the Turtlebot 3 in square!")
        self.run_bb8()


if __name__ == '__main__':
    rospy.init_node('ttb3_move', anonymous=True)
    obj = Move_BB8(0)
    try:
        obj.move_bb8()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
