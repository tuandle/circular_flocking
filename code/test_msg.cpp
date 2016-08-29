#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <vector>
#include <circular_flocking/vel.h>
int main(int argc, char** argv){

	//Initialize ROS
	ros::init(argc,argv,"test_msgs");
	ros::NodeHandle nh;
	ros::Rate rate(10);
	ros::Publisher cmd_vel_pub_ = nh.advertise<circular_flocking::vel>("/irobot1/cmd_vel",1000);
	/*
	std::vector<float> d;
	d.assign(2,0);
	float* p = d.data();
	*p = 105;
	p++;
	*p = 200;
	*/
	double vel_x = 15;
	double vel_y = 25;
	while (nh.ok()){
		circular_flocking::vel vel_;
		vel_.left = vel_x;
		vel_.right = vel_y;
		cmd_vel_pub_.publish(vel_);

		ros::spinOnce();
		rate.sleep();
	}
	
	return 0;
}