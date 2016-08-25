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
#include <PID/PID.h>
using namespace std;

int main(int argc, char** argv){

	//Initialize ROS
	ros::init(argc,argv,"irobot1_init");
	ros::NodeHandle nh;
	SpeedController move(nh);
	//End of initializing ROS


	move.GetToGoal_pid(-1.5,-1,3);

	
	return 0;
}