#ifndef _SpeedController_h 
#define _SpeedController_h

#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/tf.h>
#include <nr3/nr3.h>
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
		void vel2_info(const geometry_msgs::Twist&);
		void vel3_info(const geometry_msgs::Twist&);
		
		double u_tf(double arr[], double, double, double arr_v[], double, double, double); // position,theta,v_neighbor,psi,ki,gi
		//double u_p_t(double arr[], double, double, double); // 3 robots
		double u_p_t(double arr[], double, double, double); // 2 robots
		double u_linear(double arr[], double, double);
		double w_p_t(double arr[], double, double);  // 2robots
		//double w_p_t(double arr[], double, double, double,double);// 3 robots
		double w_linear(double arr[], double, double);
		void Flock(double,double,double,double);
		void linear_flock(double, double, double, double, double);
		void position_print();
		template<typename Method, typename F, typename Float>
		double integrate(F f, Float a, Float b, int steps, Method m){
			double s = 0;
			double h = (b-a)/steps;
			for (int i = 0; i < steps; i++)
				s += m(f, a+h*i, h);
			return h*s;
		}

		class simpson{
			public:
				template<typename F, typename Float>
				double operator()(F f, Float x, Float h) const{
					return (f(x) + 4*f(x+h/2) + f(x+h)/6);
				}
		};
	private:	
		ros::NodeHandle nh_;
		ros::Publisher cmd_vel_pub_;
		ros::Subscriber vel_irobot2,vel_irobot3;
		tf::TransformListener listener_, listener1_, listener2_, listener3_, vl2_, vl3_;
		double pose_x, pose_y, pose_theta, vn_2, vn_3;
		double vel_neighbor[2];
		double max_speed = 500;
};
#endif