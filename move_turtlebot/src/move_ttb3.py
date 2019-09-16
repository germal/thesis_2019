#! /usr/bin/env python


# This program will just publish the linear and angular velocity in a way that
# it will make the robot move in square until the node is stopped.

import rospy
import sys
import roslib

import cv2

import numpy as np



from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


from nav_msgs.msg import Odometry


class Move_BB8():



    def __init__(self, loop):
        print ("Constructor for class")
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.scanCB)
       # to store sensor datas
        self.right_sensor = 0
        self.left_sensor = 0
        self.front_sensor = 0
        self.front_right = 0
        self.front_left = 0

        self.activate = 0


        self.start_time = rospy.Time.now()



        self.turn_flag = 0

        self.count = 0


        self.rate = rospy.Rate(10) # 10Hz freq
        self.loop = loop
        self.ctrl_c = False #  This is an indicator whether user has terminated the program yet or not/
        self.vel = Twist() # the variable with the type Twist
        self.vel.linear.x = self.loop


        self.testGlobal = 0

        rospy.on_shutdown(self.shutdownhook) # this gets triggered on shutdown



    def scanCB(self, data):

       # Reference : http://www.theconstructsim.com/wall-follower-algorithm/

        # 360 data points in scandata.ranges
        scanarray = data.ranges

       # Defining the ranges of scan array
        self.front_sensor = (sum(scanarray[355:360])+sum(scanarray[0:5]))/10
        self.front_left = (sum(scanarray[80:90]))/10
        self.front_right = (sum(scanarray[270:280]))/10

        print(' Left : ' ,self.front_left)
        print(' Right : ' ,self.front_right)
        print(' Front : ', self.front_sensor )

        # This callback configures how thw right wall following is decided.

        # We need to find the wall when 1. There is no detection of obstacles
        # 2. Obstacles is present on the left side
        # 3. Obstacles is presetn on both left and right side
        # We need to turn left when 1. Obstacles in front of the robot
        #  2. obstacels in both front and left 3. obstacles in both front and right
        #  4. obstacles in front,left,right

       # we only follow the wall when obstacles is detected only on the right side (since this is right wall following)

        if self.front_sensor > 0.8 and self.front_left > 0.6 and self.front_right > 0.6:
            # find the wall
            self.vel.linear.x = 0.05
            self.vel.angular.z = -0.1
            self.pub.publish(self.vel)
        elif self.front_sensor < 0.8 and self.front_left > 0.6 and self.front_right > 0.6:
            # turn left
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.1
            self.pub.publish(self.vel)

        elif self.front_sensor > 0.8 and self.front_left > 0.6 and self.front_right < 0.6:
            # follow the right side of the wall
            self.vel.linear.x = 0.05
            self.vel.angular.z = 0
            self.pub.publish(self.vel)
        elif self.front_sensor > 0.8 and self.front_left < 0.6 and self.front_right > 0.6:
            # find the wall
            self.vel.linear.x = 0.05
            self.vel.angular.z = -0.1
            self.pub.publish(self.vel)
        elif self.front_sensor < 0.8 and self.front_left > 0.6 and self.front_right < 0.6:
            # turn left
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.1
            self.pub.publish(self.vel)
        elif self.front_sensor < 0.8 and self.front_left < 0.6 and self.front_right > 0.6:
            # turn left
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.1
            self.pub.publish(self.vel)
        elif self.front_sensor < 0.8 and self.front_left < 0.6 and self.front_right < 0.6:
            # turn left
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.1
            self.pub.publish(self.vel)
        elif self.front_sensor > 0.8 and self.front_left < 0.6 and self.front_right < 0.6:
            # find the wall
            self.vel.linear.x = 0.05
            self.vel.angular.z = -0.1
            self.pub.publish(self.vel)
        else:
            # Unexpeceted case, stop the robot and print the log
            self.vel.linear.x = 0
            self.vel.angular.z = 0
            self.pub.publish(self.vel)
            print ('there is an error in the wall-following algorithm, please retry')







        # print (rospy.Time.now()-self.start_time).to_sec()
        if ((rospy.Time.now()-self.start_time).to_sec() > 100):
            self.ctrl_c = True









    def run_bb8(self):

            while not self.ctrl_c:
                connections = self.pub.get_num_connections()
                if connections > 0 and self.activate == 1:


                        # if it has done three loops stop moving
                    if (self.count > 15):  # it will turn N times : count = 8*N - 2
                        self.vel.linear.x = 0
                        self.vel.angular.z = 0
                        self.pub.publish(self.vel)
                        rospy.loginfo("Done looping")
                        self.turn_flag = 5
                        # go forward for about 1 metre
                    elif (self.turn_flag ==0):
                        self.vel.angular.z = 0
                        self.vel.linear.x = 0.24
                        self.count = self.count + 1
                        self.pub.publish(self.vel)
                        rospy.loginfo("Straight Cmd Published")
                        rospy.sleep(4)

                        self.turn_flag = 1
                    # turn left 90 degrees
                    elif (self.turn_flag == 1):
                        self.vel.linear.x = 0
                        self.vel.angular.z = 0.3125
                        self.count = self.count + 1
                        self.pub.publish(self.vel)
                        rospy.loginfo("Turning Cmd Published")
                        rospy.sleep(5)
                        self.turn_flag = 0










    def shutdownhook(self):
        # works betsruter than the rospy.is_shutdown()
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
