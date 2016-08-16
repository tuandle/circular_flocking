#ifndef _SpeedController_h 
#define _SpeedController_h

#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/tf.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <algorithm>

class SpeedController {
	public:
		SpeedController(ros::NodeHandle &nh);
		~SpeedController();

		void GetToGoal(double, double, bool);
		void GetToGoal_pid(double, double, double);
		double set_pose();
		double get_pose() const;
		double var_theta(double, double); // var_theta_const, integral omega
		double var_phi(double, double, double); //A, x, y
		//double psi(double, double); //theta, var_theta
		//double satsm(double,double,double); // linear velocity, const1, const2
		double uctr(double,double,double,double,double); // var_phi, r, x, y, theta, linear velocity, psi
		double tau(double, double, double, double,double);//var_phi, r, x, y, theta, linear v, psi, satm, const1, const2
		double omega(double,double,double,double);//x, y, theta, linear v, r
		void Move_Han(double,double,double,double,double); // x0, y0, theta_0,v_0, omega_0,
		//void ComputeError();
		//void setKpKiKd();

		double sigma_func(double);
		double beta_i(double, double, double, double);
		double r_i(double, double);
		double r_ij(double, double, double, double);
		double var_phi0(double, double, double, double); //A1, A2, x, y
		double var_phi1(double, double, double, double,double, double); //A1, A2, x, y
		double rho_i(double arr[], double);
		double s_i(double arr[], double, double, double); // positons, theta, k, g
		double gamma_i(double, double, double); // x, y, theta
		void vel1_info(const geometry_msgs::Twist&);
		void vel3_info(const geometry_msgs::Twist&);
		double u_tf(double arr[], double, double, double arr_v[], double, double, double); // position,theta,v_neighbor,psi,ki,gi
		void Flock(double,double,double,double);
	
	private:	
		ros::NodeHandle nh_;
		ros::Publisher cmd_vel_pub_;
		ros::Subscriber vel_irobot1,vel_irobot3;
		tf::TransformListener listener_, listener1_, listener2_, listener3_,vl1_, vl3_;
		double pose_x, pose_y, pose_theta, v_n;
		double vel_neighbor[2];

};
#endif