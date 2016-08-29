#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <vector>

void cmd_cb(const std_msgs::Float32MultiArray& vel_cb){
	//std::vector<float> d = vel_cb->data;
	//float* p = d.data();
	std::cout <<"Left wheel: " << vel_cb.data[0] << "\n";
	//p++;
	//std::cout <<"Right wheel: " << *p << "\n";
}

int main(int argc, char** argv){

	//Initialize ROS
	ros::init(argc,argv,"test_sub_msgs");
	ros::NodeHandle nh;
	ros::Rate rate(10);
	ros::Subscriber sub = nh.subscribe("/irobot1/cmd_vel",100,cmd_cb);
	ros::spin(); 
}