/**
 * ROSCPP demo publisher.
 * Sends twist messages for controlling a robot base.
 * 
 * Junaed Sattar <jsattar@clarkson.edu>
 * February 2015.
 *
 */

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sstream>

/**
 *  
 * This tutorial demonstrates simple sending of messages over the ROS system
 * and controlling a robot.
 *   
 */
int main(int argc, char **argv)
{
	/// Name your node
	ros::init(argc, argv, "stage_mover");
	/// Every ros node needs a node handle, similar to your usual  file handle.
	ros::NodeHandle nh_;
	/// Publisher object that decides what kind of topic to publish and how fast.
	ros::Publisher cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	// We will be sending commands of type "twist"
	geometry_msgs::Twist base_cmd;
	// User input
	char cmd[50];

	std::cout << "Type a command and then press enter.  "
		"Use 'f' to move forward, 'l' to turn left, "
		"'r' to turn right, '.' to exit.\n";

	/// The main loop will run at a rate of 10Hz, i.e., 10 times per second.
	ros::Rate loop_rate(10);
	/// Standard way to run ros code. Will quite if ROS is not OK, that is, the master is dead.
	while (ros::ok())
	{
		std::cin.getline(cmd, 50);
		if(cmd[0]!='f' && cmd[0]!='l' && cmd[0]!='r' && cmd[0]!='.')
		{
			std::cout << "unknown command:" << cmd << "\n";
			continue;
		}

		base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;

		//move forward
		if(cmd[0]=='f'){
			base_cmd.linear.x = 0.25;
			base_cmd.angular.z = 0;
		} 
		//turn left (yaw) and drive forward at the same time
		else if(cmd[0]=='l'){
			base_cmd.angular.z = 0.75;
			base_cmd.linear.x = 0.25;
		} 
		//turn right (yaw) and drive forward at the same time
		else if(cmd[0]=='r'){
			base_cmd.angular.z = -0.75;
			base_cmd.linear.x = 0.25;
		} 
		//quit
		else if(cmd[0]=='.'){
			break;
		}
		/// Here's where we publish the actual message.
		cmd_vel_pub_.publish(base_cmd);
		/// Spin the ros main loop once 
		ros::spinOnce();
		/// Sleep for as long as needed to achieve the loop rate.
		loop_rate.sleep();
	}
	return 0;
}

