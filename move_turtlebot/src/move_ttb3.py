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




    def __init__(self,exploration_method, loop, duration):
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
        self.relocalization_flag = 1
        self.kidnap_flag = 0
        self.cuser_select = 0

        self.menu_select = exploration_method
        self.explore_duration = duration
        self.num_loops = loop

        # 1 for the square  2 for the right wall following
        print('Mode selected ', self.menu_select)


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
        # Goal pose is defined in the frame of camera_odom_frame
        self.goal.target_pose.header.frame_id = 'camera_odom_frame'
        # Set the time as now. Update this later
        self.goal.target_pose.header.stamp = rospy.Time.now()


       # to store sensor datas
        self.right_sensor = 0
        self.left_sensor = 0
        self.front_sensor = 0
        self.front_right = 0
        self.front_left = 0

       # Init other variables.
        self.start_time = rospy.Time.now()
        self.turn_flag = 0
        self.count = 0
        self.rate = rospy.Rate(10) # 10Hz freq
        self.loop = loop
        self.ctrl_c = False #  This is an indicator whether user has terminated the program yet or not/
        self.vel = Twist() # the variable with the type Twist
        self.vel.linear.x = self.loop
        rospy.on_shutdown(self.shutdownhook) # this gets triggered on shutdown




    def odomCB(self, data):


        if (data.pose.covariance[0] > 0.1): # if the tracking confidence above 0.1
            self.kidnap_flag = 1           # Flag that the kidnap event has happened.
            print('The robot has been kidnapped')
            self.relocalization_flag = 0
        #    print('Cancelling move_base goal due to kidnapping') # If the navigation stack was in progress, preempt the goal.
            self.move_base.cancel_goal()

        # If it is the first time running this, take the current position as the old pose.
        if (self.read_first_odom == 0):
            self.distance = 0.01
            self.old_x = data.pose.pose.position.x
            self.old_y = data.pose.pose.position.y
            self.old_z = data.pose.pose.position.z
            self.read_first_odom = 1
        else:

           # Calculating the difference of the consecutive poses.
            self.distance = math.sqrt((data.pose.pose.position.x - self.old_x)**2 +(data.pose.pose.position.y - self.old_y)**2 + (data.pose.pose.position.z - self.old_z)**2)
            self.old_x = data.pose.pose.position.x
            self.old_y = data.pose.pose.position.y
            self.old_z = data.pose.pose.position.z

            if self.distance > 2: # If the jump pose is observed, then set up the relocalization flag
                print(self.distance)
                print('Relocalization event occured!!!')
                self.relocalization_flag = 1
                self.kidnap_flag = 0
                # Go back to the base
                self.csend_goal()



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
       # it is assumed that during the mappign phase, the camera won't be kidnapped.
        if   ((rospy.Time.now()-self.start_time).to_sec() < self.explore_duration and self.menu_select ==2) or self.cuser_select ==1:
            # if the robot is kidnapped, run the right wall following to

            if self.front_sensor > 0.8 and self.front_left > 0.65 and self.front_right > 0.65:
                # find the wall
                self.vel.linear.x = 0.05
                self.vel.angular.z = -0.1
                self.pub.publish(self.vel)
            elif self.front_sensor < 0.8 and self.front_left > 0.65 and self.front_right > 0.65:
                # turn left
                self.vel.linear.x = 0.0
                self.vel.angular.z = 0.1
                self.pub.publish(self.vel)

            elif self.front_sensor > 0.8 and self.front_left > 0.65 and self.front_right < 0.65:
                # follow the right side of the wall
                self.vel.linear.x = 0.05
                self.vel.angular.z = 0
                self.pub.publish(self.vel)
            elif self.front_sensor > 0.8 and self.front_left < 0.65 and self.front_right > 0.65:
                # find the wall
                self.vel.linear.x = 0.05
                self.vel.angular.z = -0.1
                self.pub.publish(self.vel)
            elif self.front_sensor < 0.8 and self.front_left > 0.65 and self.front_right < 0.65:
                # turn left
                self.vel.linear.x = 0.0
                self.vel.angular.z = 0.1
                self.pub.publish(self.vel)
            elif self.front_sensor < 0.8 and self.front_left < 0.65 and self.front_right > 0.65:
                # turn left
                self.vel.linear.x = 0.0
                self.vel.angular.z = 0.1
                self.pub.publish(self.vel)
            elif self.front_sensor < 0.8 and self.front_left < 0.65 and self.front_right < 0.65:
                # turn left
                self.vel.linear.x = 0.0
                self.vel.angular.z = 0.1
                self.pub.publish(self.vel)
            elif self.front_sensor > 0.8 and self.front_left < 0.65 and self.front_right < 0.65:
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
      # If the time has passed over the set duration then go to the menu.
        elif ((rospy.Time.now()-self.start_time).to_sec() > self.explore_duration and self.menu_select ==2) and self.cuser_select == 0:
            self.select_menu() # go to menu selection








    def run_bb8(self):
            # This function will run infinitely until the user decides to terminate program
            # by pressing Ctrl + C
            while not self.ctrl_c:

                connections = self.pub.get_num_connections()


                # If the robot has been opearting more than n seconds, go back to charging station.
                if ((rospy.Time.now()-self.start_time).to_sec() > 150 and (rospy.Time.now()-self.start_time).to_sec() < 152 ):
                    self.vel.linear.x = 0
                    self.vel.angular.z = 0
                    self.pub.publish(self.vel)
                    print('Battery Low, going to the charging base')
                    self.csend_goal()
                    self.cuser_select = 3 # Stop the wall following.
                    self.turn_flag = 5 # Stop the square movement.


                if connections > 0 and self.menu_select == 1: # This is only used for mapping.


                        # if it has done three loops stop moving
                    if (self.count > (self.num_loops*8-1)):  # it will loop N times : count = 8*N - 1
                        self.vel.linear.x = 0
                        self.vel.angular.z = 0
                        self.pub.publish(self.vel) # Stop the robot
                        rospy.loginfo("Done looping")
                        self.menu_select = 0 # stop mapping
                        self.turn_flag = 5
                        print(((rospy.Time.now()-self.start_time).to_sec()))
                        self.select_menu() # Go to menu
                        # go forward for about 1 metre
                    elif (self.turn_flag ==0):
                        self.vel.angular.z = 0
                        self.vel.linear.x = 0.20
                        self.count = self.count + 1
                        self.pub.publish(self.vel)
                        rospy.loginfo("Straight Cmd Published")
                        self.turn_flag = 1
                        rospy.sleep(5)


                    # turn left 90 degrees
                    elif (self.turn_flag == 1):
                        self.vel.linear.x = 0
                        self.vel.angular.z = 0.1953
                        self.count = self.count + 1
                        self.pub.publish(self.vel)
                        rospy.loginfo("Turning Cmd Published")
                        self.turn_flag = 0
                        rospy.sleep(8)



    def select_menu(self):

   # Stop the robot and let the user to choose menu.
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        self.pub.publish(self.vel)

        print('Mapping has finished, Choose 1. Keep Exploring 2. Go to base')
        self.cuser_select = input()
        if self.cuser_select == 1:
            print ('Keep Exploring')
        elif self.cuser_select == 2:
            print ('Going to base')
            self.csend_goal()

    def csend_goal(self):


        # Move the robot to charging base
        if self.kidnap_flag == 1: # If the robot is kidnapped
            print('Goal cancelled')
            self.goal.target_pose.header.stamp = rospy.Time.now()
            self.move_base.cancel_goal() # If it is kidnapped, cancel the goal
            self.cuser_select = 1 # reactivate right wall following.


        if self.relocalization_flag == 1 and (self.cuser_select == 3 or self.cuser_select ==2)  : # If the robot is relocalized then, keep move to move_base
            print('Sending a base goal')
            self.goal.target_pose.header.stamp = rospy.Time.now()
            self.move_base.send_goal(self.goal)
            # Sometimes error Received comm state PREEMPTING when in simple state DONE with SimpleActionClient
            # would occur.
            #self.move_base.wait_for_result()
            #self.move_base.get_result()


    def shutdownhook(self):
        # works betsruter than the rospy.is_shutdown()
        print('The total time of operation equals to ')
        print (rospy.Time.now()-self.start_time).to_sec()
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        self.pub.publish(self.vel)
    #   self.move_bb8(); # move the bot with updated Twist
        self.ctrl_c = True

    def move_bb8(self):


        rospy.loginfo("Starting the node")
        self.run_bb8()






if __name__ == '__main__':
    rospy.init_node('ttb3_move', anonymous=True)
    print('Please choose your mapping method 1. 1m x 1m Square Loop 2. Right Wall Following')
    method_choice = input()
    if method_choice == 1:
        print('How many square loops would you like to do??')
        user_loops = input()
        obj = Move_BB8(method_choice,user_loops,0)
    elif method_choice ==2:
        print('How many seconds would you like the wall-following algorithm to be running?')
        user_duration = input()
        obj = Move_BB8(method_choice,0,user_duration)
    try:
        obj.move_bb8()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
