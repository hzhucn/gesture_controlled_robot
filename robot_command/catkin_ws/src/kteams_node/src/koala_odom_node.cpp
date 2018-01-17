#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

double vlin;
double vth;

void wheel_speed_callback(const geometry_msgs::TwistStamped::ConstPtr& wheel_speed_msg) {
  vlin = wheel_speed_msg->twist.linear.x;
  vth = wheel_speed_msg->twist.angular.z;
}

int main(int argc, char * argv[]) {

  ros::init(argc, argv, "koala_odom");
  ros::NodeHandle nh;

  ros::Publisher odometry_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);
  ros::Subscriber wheel_speed_subscriber = nh.subscribe<geometry_msgs::TwistStamped>("perception_wheel_speed", 1, &wheel_speed_callback);
  tf::TransformBroadcaster odom_broadcaster;


  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  vlin = 0.0;
  vth = 0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  
  ros::Rate loop_rate(25);

  while(nh.ok()) {

    // Check for new messages
    ros::spinOnce();

    // Update the pose estimate of the robot
    current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    x += dt * vlin * cos(th);
    y += dt * vlin * sin(th);
    th += dt * vth;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);


    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);



    // We now set up the Odometry message
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    // Set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    // Set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vlin;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = vth;
    odom.twist.twist.angular.y = vth;
    odom.twist.twist.angular.z = vth;

    // Publish the message
    odometry_pub.publish(odom);
    
    last_time = current_time;
    
    loop_rate.sleep();
  }
}
