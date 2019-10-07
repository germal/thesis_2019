#! /usr/bin/env python


# This program will just publish the linear and angular velocity in a way that
# it will make the robot move in square until the node is stopped.

import rospy
import sys
import roslib
import math
import actionlib
import cv2

import numpy as np



from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class Move_BB8():




    def __init__(self, loop):
#        print ("Constructor for class")
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.scanCB)
        self.sub2 = rospy.Subscriber('/camera/odom/sample', Odometry, self.odomCB)

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Connecting to the move_base server")

    #    self.move_base.wait_for_server(rospy.Duration(10)) # wait 10 seconds for the server

     # To calculate the distance between old and current pose.
        self.old_x = 0
        self.old_y = 0
        self.old_z = 0
        self.read_first_odom = 0
        self.distance = 0
        self.relocalization_flag = 0
        self.kidnap_flag = 0
        self.cuser_select = 0


       # Init goal.
        self.goal = MoveBaseGoal()
        # Set the default position at 0 0 0 , 0 0 0 1
        self.goal.target_pose.pose.position.x = 0.0
        self.goal.target_pose.pose.position.y = 0.0
        self.goal.target_pose.pose.position.z = 0.0
        self.goal.target_pose.pose.orientation.x = 0
        self.goal.target_pose.pose.orientation.y = 0
        self.goal.target_pose.pose.orientation.z = 0
        self.goal.target_pose.pose.orientation.w = 1

        # Use the map frame to define goal poses
        self.goal.target_pose.header.frame_id = 'camera_odom_frame'

        # Set the time as now. Update this later
        self.goal.target_pose.header.stamp = rospy.Time.now()

        # You can send a goal like this
    #    self.move_base.send_goal(self.goal)



       # to store sensor datas
        self.right_sensor = 0
        self.left_sensor = 0
        self.front_sensor = 0
        self.front_right = 0
        self.front_left = 0

        self.menu_select = loop
        print('Mode selected ', self.menu_select)


        self.start_time = rospy.Time.now()
        self.count_time = rospy.Time.now()



        self.turn_flag = 0

        self.count = 0


        self.rate = rospy.Rate(10) # 10Hz freq
        self.loop = loop
        self.ctrl_c = False #  This is an indicator whether user has terminated the program yet or not/
        self.vel = Twist() # the variable with the type Twist
        self.vel.linear.x = self.loop


        self.testGlobal = 0

        rospy.on_shutdown(self.shutdownhook) # this gets triggered on shutdown




    def odomCB(self, data):

        if (data.pose.covariance[0] > 0.1):
            self.kidnap_flag = 1
        # If it is the first time running this, take the current position as the old pose.
        if (self.read_first_odom == 0):
            self.distance = 0.01
            self.old_x = data.pose.pose.position.x
            self.old_y = data.pose.pose.position.y
            self.old_z = data.pose.pose.position.z
        else:

           # Calculating the difference of the consecutive poses.
            self.distance = math.sqrt((data.pose.pose.position.x - self.old_x)**2 +(data.pose.pose.position.y - self.old_y)**2 + (data.pose.pose.position.z - self.old_z)**2)
            self.old_x = data.pose.pose.position.x
            self.old_y = data.pose.pose.position.y
            self.old_z = data.pose.pose.position.z
            if self.distance > 1:
                print(self.distance)
                self.relocalization_flag = 1



    def scanCB(self, data):

       # Reference : http://www.theconstructsim.com/wall-follower-algorithm/

        # 360 data points in scandata.ranges
        scanarray = data.ranges

       # Defining the ranges of scan array
        self.front_sensor = (sum(scanarray[355:360])+sum(scanarray[0:5]))/10
        self.front_left = (sum(scanarray[80:90]))/10
        self.front_right = (sum(scanarray[270:280]))/10



        # This callback configures how thw right wall following is decided.

        # We need to find the wall when 1. There is no detection of obstacles
        # 2. Obstacles is present on the left side
        # 3. Obstacles is presetn on both left and right side
        # We need to turn left when 1. Obstacles in front of the robot
        #  2. obstacels in both front and left 3. obstacles in both front and right
        #  4. obstacles in front,left,right

       # we only follow the wall when obstacles is detected only on the right side (since this is right wall following)

        if   self.kidnap_flag == 1 or self.cuser_select == 2: # if the robot is kidnapped, run the right wall following to

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



    def run_bb8(self):



            while not self.ctrl_c:
                connections = self.pub.get_num_connections()
                if connections > 0 and self.menu_select == 1: # This is used for mapping.


                        # if it has done three loops stop moving
                    if (self.count > 22):  # it will loop N times : count = 8*N - 2
                        self.vel.linear.x = 0
                        self.vel.angular.z = 0
                        self.pub.publish(self.vel)
                        rospy.loginfo("Done looping")
                        self.menu_select = 0 # stop mapping
                        self.turn_flag = 5
                        self.select_menu()
                        # go forward for about 1 metre
                    elif (self.turn_flag ==0):
                        self.vel.angular.z = -0.24
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










    def select_menu(self):
        print('Choose your input 1. Mappin 2. Explore 3. Go back to base')
        self.cuser_select = input()

        if self.cuser_select == 1:
            self.menu_select = 1
            self.turn_flag = 0
            self.count = 0
            self.run_bb8()

        elif self.cuser_select == 2:
            print ('start right wall following')

        elif self.cuser_select == 3:
            self.csend_goal()

    def csend_goal(self):
        self.move_base.send_goal(self.goal)

    def shutdownhook(self):
        # works betsruter than the rospy.is_shutdown()
        print('The time of operation equals to ')
        print (rospy.Time.now()-self.start_time).to_sec()
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        self.pub.publish(self.vel)
    #   self.move_bb8(); # move the bot with updated Twist
        self.ctrl_c = True

    def move_bb8(self):
        if (self.menu_select == 1):
            rospy.loginfo("Moving the Turtlebot 3 in square!")
            self.run_bb8()
        elif (self.menu_select == 2):
            self.select_menu()





if __name__ == '__main__':
    rospy.init_node('ttb3_move', anonymous=True)
    print('Please choose your input 1. Mapping (use discrete square loops) 2. Look for other options ')

    user_select = input()
    obj = Move_BB8(user_select)
    try:
        obj.move_bb8()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
