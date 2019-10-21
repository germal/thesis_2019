# thesis_2019

Start up:
On the TTB3

$ ssh pi@192.168.43.53
$ roslaunch turtlebot3_bringup turtlebot3_robot.launch
Camera:
$ roslaunch realsense2_camera rs_t265.launch (this is w/o json)
$ roslaunch realsense2_camera my_t265.launch (w json)


On pc:
$ roslaunch turtlebot3_bringup turtlebot3_remote.launch
rosrun rviz rviz -d `rospack find turtlebot3_description`/rviz/model.rviz

Setting up tf:
$ roslaunch robot_setup_tf tf_setup.launch
Setting up move_base package:
$ roslaunch turtlebot3_navigation move_base_cam.launch

Running the final demo
$ rosrun move_turtlebot move_ttb3.py
