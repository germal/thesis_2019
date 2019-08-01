#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>


// Declaring global variables




geometry_msgs::TransformStamped odom_trans;


void camodomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_INFO("Seq: [%d]", msg->header.seq);
  ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);


  odom_trans.transform.translation.x = msg->pose.pose.position.x;
  odom_trans.transform.translation.y = msg->pose.pose.position.y;
  odom_trans.transform.translation.z = msg->pose.pose.position.z;
  odom_trans.transform.rotation.x = msg->pose.pose.orientation.x;
  odom_trans.transform.rotation.y = msg->pose.pose.orientation.y;
  odom_trans.transform.rotation.z = msg->pose.pose.orientation.z;
  odom_trans.transform.rotation.w = msg->pose.pose.orientation.w;



}




int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Subscriber odom_sub = n.subscribe("camera/odom/sample", 10, camodomCallback);
  ros::Publisher point_pub = n.advertise<geometry_msgs::TransformStamped>("tf_calibration",10);
  ros::Rate loop_rate(5);
  tf::TransformBroadcaster odom_broadcaster;

    ros::Time current_time ;
    current_time = ros::Time::now();
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    while(n.ok())
    {




    current_time = ros::Time::now();
    odom_trans.header.stamp = current_time;
    odom_broadcaster.sendTransform(odom_trans);
    point_pub.publish(odom_trans);
    ros::spinOnce();
    loop_rate.sleep();


  }

    return 0;
  }
