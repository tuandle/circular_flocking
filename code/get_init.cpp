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

using namespace std;

int main(int argc, char** argv){

	//Initialize ROS
	ros::init(argc,argv,"irobot2_init");
	ros::NodeHandle nh;
	SpeedController move(nh);
	//End of initializing ROS


	//move.GetToGoal_pid(-1,-0.6,2);
	move.GetToGoal_pid(-1.1,-0.5,M_PI/8);
	
	return 0;
}