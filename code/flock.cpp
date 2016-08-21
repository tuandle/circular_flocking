#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <SpeedController/SpeedController.h>

int main(int argc, char** argv){
	ros::init(argc,argv,"irobot2_control");
	ros::NodeHandle nh;
	SpeedController flocking(nh);
	/*
	ros::Rate rate(10);
	tf::TransformListener listener;

	listener.waitForTransform("/world","/irobot2",ros::Time(0),ros::Duration(1));
	tf::StampedTransform transform;

	double current_roll, current_pitch, current_yaw, current_x, current_y;
	while (nh.ok()){
		try{
			listener.lookupTransform("/world","/irobot2",ros::Time(0),transform);
			transform.getBasis().getRPY(current_roll, current_pitch, current_yaw);
			current_x = transform.getOrigin().x();
			current_y = transform.getOrigin().y();
			std::cout << "Current position: x - "<<current_x<<" y - "<<current_y<<" yaw - "<<current_yaw << "\n";

		}
		catch(tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}
		ros::spinOnce();
		rate.sleep();
	}
	*/
	flocking.Flock(-1,-0.6,2,0);
	//flocking.tf_debug();
	flocking.linea_flock(-1.1,-0.5,M_PI/8,0,0);
}