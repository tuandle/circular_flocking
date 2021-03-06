#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/tf.h>
#include <math.h>
//#include <boost/numeric/odeint.hpp>
#include <geometry_msgs/Twist.h>
#include <SpeedController/SpeedController.h>
//#include <PID/PID.h>
 
using namespace std;
 
SpeedController::SpeedController(ros::NodeHandle &nh){
    nh_ = nh;
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/irobot2/cmd_vel",1000);
    vel_neighbor[2]={ };
    //vel_irobot1 = nh_.subscribe("/irobot1/cmd_vel",1000,&SpeedController::vel1_info,this);
    //vel_irobot3 = nh_.subscribe("/irobot3/cmd_vel",1000,&SpeedController::vel3_info,this);
}
 
SpeedController::~SpeedController(){}
 
void SpeedController::GetToGoal(double v_linear, double v_angular, bool done){
     
    ros::Rate rate(10);
    listener_.waitForTransform("/world", "/irobot2", ros::Time(0), ros::Duration(1));
    tf::StampedTransform transform_;
 
    while (nh_.ok()){
        try{
            listener_.lookupTransform("/world", "/irobot2", ros::Time(0), transform_);//listen to current frame
            double current_roll,current_pitch,current_yaw; //get current yaw
            transform_.getBasis().getRPY(current_roll,current_pitch,current_yaw); 
            double current_x,current_y; //get current positions
            current_x = transform_.getOrigin().x();
            current_y = transform_.getOrigin().y();
             
            geometry_msgs::Twist vel_;
 
            /*
            double distance_to_goal = sqrt(pow(goal_x - current_x,2)+pow(goal_y - current_y,2)); // need to move ?
            double diff_yaw = goal_yaw - current_yaw; // need to turn?
            ROS_INFO_STREAM("Current x: " << current_x);
            ROS_INFO_STREAM("Current x: " << goal_x);
            ROS_INFO_STREAM("Distance to goal: " << distance_to_goal);
             
            //We need to move to our goal
            //Compute velocities
            if (abs(distance_to_goal) > 0.1){
                vel_.linear.x = 0.1;
                vel_.angular.z = 0.1;
            }
            else{
                vel_.linear.x = 0;
                vel_.angular.z = 0; 
            }
            //End of computing velocities
            */
            if (!done){
                vel_.linear.x = v_linear;
                vel_.angular.z = v_angular;
            }
            else{
                vel_.linear.x = 0;
                vel_.angular.z = 0; 
            }
            cmd_vel_pub_.publish(vel_); //publish velocities
        }
        catch(tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        ros::spinOnce();
        rate.sleep();
    }
}
 
/*     
void SpeedController::GetToGoal_pid(double x_goal, double y_goal, double yaw_goal){
    double Kp = 1, Ki = 0.01, Kd = 0.01;
    bool init = true;
    double SetPoint_linear = sqrt(pow(x_goal,2) + pow(y_goal,2)), SetPoint_angular = yaw_goal;
    double Input_linear, Output_linear, Input_angular;
    double Output_angular;
 
    PID pid_linear(&Input_linear, &Output_linear, &SetPoint_linear, Kp, Ki, Kd);
    pid_linear.SetOutputLimit(0,0.5);
     
    ros::Rate rate(100);
    listener_.waitForTransform("/world", "/irobot2", ros::Time(0), ros::Duration(1));
    tf::StampedTransform transform_;
     
    while (nh_.ok()){
        try{
            listener_.lookupTransform("/world", "/irobot2", ros::Time(0), transform_);//listen to current frame
            double current_roll,current_pitch,current_yaw; //get current yaw
            transform_.getBasis().getRPY(current_roll,current_pitch,current_yaw); 
            double current_x,current_y; //get current positions
            current_x = transform_.getOrigin().x();
            current_y = transform_.getOrigin().y();
 
            Input_angular = current_yaw;
 
            geometry_msgs::Twist vel_;
             
            double distance_to_goal = pow((pow((x_goal - current_x),2)+pow((y_goal - current_y),2)),0.5); // need to move ?
            Input_linear = distance_to_goal;
             
            double diff_yaw = yaw_goal - current_yaw; // need to turn?
             
            ROS_INFO_STREAM("Current x: " << current_x);
            ROS_INFO_STREAM("Goal x: " << SetPoint_linear);
            ROS_INFO_STREAM("Distance to goal: " << distance_to_goal);
            ROS_INFO_STREAM("Yaw goal: " << yaw_goal);
            ROS_INFO_STREAM("Current yaw: " << current_yaw);
            ROS_INFO_STREAM("Diff in yaw: " << diff_yaw);
             
            if (abs(distance_to_goal) > 0.1){
                Output_angular = 10*(atan2((y_goal-current_y),(x_goal-current_x))-current_yaw);
                vel_.angular.z = Output_angular;
                cout << "Angular velocity: " << vel_.angular.z << "\n";
 
                pid_linear.Compute();
                //vel_.linear.x = 0.3;
                vel_.linear.x = Output_linear;
                cout << "Linear velocity: " << vel_.linear.x << "\n";
                //pid_angular.Compute();
                //vel_.angular.z = Output[1];
                 
            }
            else{
                //vel_.linear.x = 0;
                if (abs(diff_yaw) < 0.01){
                    vel_.angular.z = 0;
                }
            } 
 
            cmd_vel_pub_.publish(vel_); //publish velocities
        }
        catch(tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        ros::spinOnce();
        rate.sleep();
    }
}
*/  
double SpeedController::omega(double x, double y, double theta, double v){
    double r = sqrt(pow(x,2)+pow(y,2));
    return ((x*sin(theta) - y*cos(theta)) * v / pow(r,2)); 
}

double SpeedController::var_phi(double A, double x, double y){
    double r = sqrt(pow(x,2)+pow(y,2));
    if ((r<=0.5) || (r == 1) || (r>=1.5))
        return (0);
    else
        if ((r>0.5)&&(r<1))
            return (-A * exp(-pow(r-0.75,2)/(pow(0.25,2)-pow(r-0.75,2))));
        else
            if ((r > 1)&&(r<1.5))
                return (A * exp(-pow(r-1.25,2)/(pow(0.25,2)-pow(r-1.25,2))));
}
 
double SpeedController::uctr(double x, double y, double theta, double psi_t, double v){
    double A = 1.40;
    double r = sqrt(pow(x,2)+pow(y,2));
    return ((-var_phi(A,x,y)/r)*(x*cos(theta)+y*sin(theta)) + (var_phi(A,x,y)/r)*(-x*sin(theta)+y*cos(theta))*(1/(1+pow(v,2)))*psi_t/(1+pow(psi_t,2))) - tanh(v-0.5);
}
 
double SpeedController::tau(double x, double y, double theta, double psi_t, double v){
    double A = 1.40;
    double r = sqrt(pow(x,2)+pow(y,2));
    return ((-var_phi(A,x,y)/r)*(-x*sin(theta)+y*cos(theta))*(v/(1+pow(v,2)))*1/(1+pow(psi_t,2))-tanh(psi_t));
}
 
void SpeedController::Move_Han(double x0, double y0, double theta0, double w0, double v0){
    ros::Rate rate(1000);
    listener_.waitForTransform("/world", "/irobot1", ros::Time(0), ros::Duration(1));
    tf::StampedTransform transform_;

    double dt =  0.016; // time step
    //initial conditions
    double var_theta0 = atan(y0/x0);
    double omega_t = (x0*sin(theta0)-y0*cos(theta0))*v0/(pow(x0,2)+pow(y0,2));
    
    double var_theta_t = var_theta0 + omega_t;
    double psi = theta0 - var_theta_t - M_PI/2;
    double u = uctr(x0,y0,theta0,psi,v0);
    double tt = tau(x0,y0,theta0,psi,v0);
    double w_t = w0 + tt;  
    double v_t = v0 + u; 
    //double omega_t = omega0 + tt;

    double var_theta_i1 = var_theta_t;
    double v_t_i1 = v_t;
    double w_t_i1 = w_t;
    //end of initial conditions


    ofstream fout;
    double t_start = ros::Time::now().toSec(); // starting time
    fout.open("test_12");
    fout << "v_ref: 0.39; ros rate: 100; ros sleep 0.1 second " <<"\n" ;

    while (nh_.ok()){
        try{
            double t_now = ros::Time::now().toSec(); // integrate function to this time 
            
            listener_.lookupTransform("/world", "/irobot1", ros::Time(0), transform_);//listen to current frame
            double current_roll,current_pitch,current_yaw; //get current yaw
            transform_.getBasis().getRPY(current_roll,current_pitch,current_yaw); 
            double current_x,current_y; //get current positions
            current_x = transform_.getOrigin().x();
            current_y = transform_.getOrigin().y();
            
            cout << "current v_t: " << v_t << " current angular: " << tt <<"\n";
            geometry_msgs::Twist vel_;  
            vel_.linear.x = v_t;
            vel_.angular.z = tt;
            cmd_vel_pub_.publish(vel_); //publish velocities
            
            fout << v_t << " " << tt << "\n";


            double last_w = w_t;
            double last_u = u;
            double last_tau = tt;
            double last_omega_t = omega_t;
            
            omega_t = omega(current_x,current_y,current_yaw,v_t);
            var_theta_t = var_theta_i1 + (omega_t + last_omega_t)*dt*0.5;
            
            psi = current_yaw - var_theta_t - M_PI/2;
            
            
            u = uctr(current_x,current_y,current_yaw,psi,v_t);
            tt = tau(current_x,current_y,current_yaw,psi,v_t);
 
            v_t = v_t_i1 + (u + last_u)*dt*0.5;
            cout << "new v_t: " << v_t << " new angular: " << tt <<"\n";
            w_t = w_t_i1 + (tt + last_tau)*dt*0.5;

            var_theta_i1 = var_theta_t;    
            v_t_i1 = v_t;
            w_t_i1 = w_t;
            
            double t_end = ros::Time::now().toSec();
            if ((t_end - t_start) >= 30)
                fout.close();
        }
        catch(tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(0.5).sleep();
        }
        ros::spinOnce();
        rate.sleep();
    }
}

double SpeedController::sigma_func(double s){
    return min(max(s,-0.5),0.5);
}

double SpeedController::r_i(double xi, double yi){
	return (sqrt(pow(xi,2))+pow(yi,2));
}

double SpeedController::r_ij(double xi, double yi,double xj, double yj){
	return sqrt(pow((xi-xj),2)+pow((yi-yj),2));
}

double SpeedController::var_phi0(double A1, double A2, double x, double y){
    //double r = sqrt(pow(x,2)+pow(y,2));
    double r = r_i(x,y);
    if ((r<=0.2)||((r>=0.5)&&(r<=1))||(r>=1.5))
    	return(0);
    else
    	if ((0.2<r)&&(r<0.5))
    		return(-A1*exp(-(pow((r-0.35),2))/(pow(0.15,2)-pow((r-0.35),2))));
    	else
    		if ((1<r)&&(r<1.5))
    			return(A2*exp(-(pow((r-1.25),2))/(pow(0.25,2)-pow((r-1.25),2))));
}

double SpeedController::var_phi1(double B1, double B2, double xi, double yi, double xj, double yj){
	//double r_ij = sqrt(pow((xi-xj),2)+pow((yi-yj),2));
	double rij = r_ij(xi,yi,xj,yj);
	if ((rij <= 0.4) || (rij=0.7) || (rij >=1.5))
		return(0);
	else
		if ((rij > 0.4)&&(rij < 0.7))
			return(-B1*exp(-(pow((rij-0.55),2))/(pow(0.15,2)-pow((rij-0.55),2))));
		else
			if ((rij > 0.7)&&(rij < 1.5))
				return(B1*exp(-(pow((rij-1.1),2))/(pow(0.4,2)-pow((rij-1.1),2))));
}

double SpeedController::beta_i(double x, double y, double theta, double v){
	//double r = sqrt(pow(x,2)+pow(y,2));
	double r = r_i(x,y);
	return ((x * sin(theta) - y*cos(theta))*(v/pow(r,2)));
}

double SpeedController::rho_i(double positions[6], double theta_i){
	double a1 = 10, a2 = 3, b1 = 28, b2 = 3;
	double ri = r_i(positions[0],positions[1]);
	double left = var_phi0(a1,a2,positions[0],positions[1]) * (positions[0]*cos(theta_i)+positions[1]*sin(theta_i)) / ri;
	double temp = 0;
	for (int i = 0; i < 2; i++){
		double var_rij = r_ij(positions[0],positions[1],positions[2*i+2],positions[2*i+3]);
		double var_t = ((positions[0]-positions[2*i+2])*cos(theta_i)+(positions[1]-positions[2*i+3])*sin(theta_i))/var_rij;
		temp = temp + var_phi1(b1,b2,positions[0],positions[1],positions[2*i+2],positions[2*i+3])*var_t;
	}
	return (left+temp/3);
}

double SpeedController::s_i(double positions[6], double theta_i, double ki, double gi){
	double a1 = 10, a2 = 3, b1 = 28, b2 = 3;
	double ri = r_i(positions[0],positions[1]);
	double left = ki * var_phi0(a1,a2,positions[0],positions[1]) * (-positions[0]*sin(theta_i)+positions[1]*cos(theta_i)) / ri;
	double temp = 0;
	for (int i = 0; i < 2; i++){
		double var_rij = r_ij(positions[0],positions[1],positions[2*i+2],positions[2*i+3]);
		double var_t = (-(positions[0]-positions[2*i+2])*sin(theta_i)+(positions[1]-positions[2*i+3])*cos(theta_i))/var_rij;
		temp = temp + var_phi1(b1,b2,positions[0],positions[1],positions[2*i+2],positions[2*i+3])*var_t;
	}
	return (left+gi*temp/3);
}

double SpeedController::gamma_i(double xi, double yi, double theta_i){
	double r = r_i(xi,yi);
	return ((xi * sin(theta_i) - yi*cos(theta_i))*(1/pow(r,2)));
}

void SpeedController::vel1_info(const geometry_msgs::Twist& vel_msg){
	this->vel_neighbor[0]=vel_msg.linear.x;
}

void SpeedController::vel3_info(const geometry_msgs::Twist& vel_msg){
	this->vel_neighbor[1]=vel_msg.linear.x;
}

double SpeedController::u_tf(double pos[6], double v, double theta, double v_neighbor[2], double psi_i, double ki, double gi){
	double temp_sigma = 0;
	double ri = r_i(pos[0],pos[1]);
	for (int i = 0; i < 2; i++){
    	temp_sigma = temp_sigma + sigma_func(v*gamma_i(pos[0],pos[1],theta)-v_neighbor[i]*gamma_i(pos[2*i+3],pos[2*i+4],atan(pos[2*i+4]/pos[2*i+3])));
    }
    double u = -rho_i(pos, theta) + (pos[0]*sin(theta) - pos[1]*cos(theta))*(sigma_func(psi_i)/pow(ri,2)) + s_i(pos,theta,ki,gi)*sigma_func(psi_i) - (2/3) * gamma_i(pos[0],pos[1],theta) * temp_sigma;
    return u;
}

void SpeedController::Flock(double x0, double y0, double theta0, double v0){
    
    
    ros::Rate rate(1000);
    listener_.waitForTransform("/irobot1", "/world", ros::Time(0), ros::Duration(1));
    listener2_.waitForTransform("/irobot2", "/world", ros::Time(0), ros::Duration(1));
    listener3_.waitForTransform("/irobot3", "/world", ros::Time(0), ros::Duration(1));
    tf::StampedTransform transform_, transform1_, transform3_;
    
    //double vel_neighbor[2];

    double h =  0.016; // approx. step
    //initial conditions
    double Vi_t,Ri_t,vi_l,vi_r;
    double L = 0.27, Rw = 0.065/2;

    double pos_0[6] = {x0,y0,0.2,0.4,1.0,-0.8};
    double beta_t0 = beta_i(x0,y0,theta0,v0);
    double beta_t = beta_t0;
    double var_theta0 = atan(y0/x0);
    double var_theta_t = var_theta0;
    double psi_t = theta0 - var_theta0 - M_PI/2; // t = 0;
    double vi_t = v0;
    double ri = r_i(x0,y0);
    double ki = 0.4/(1+pow(v0,2));
    double gi = 0.3/(sqrt(1+pow(v0,2)));
    double temp_sigma =0;
    for (int i = 0; i < 2; i++){
    	temp_sigma = temp_sigma + sigma_func(vi_t*gamma_i(x0,y0,theta0)-vel_neighbor[i]*gamma_i(pos_0[2*i+3],pos_0[2*i+4],atan(pos_0[2*i+4]/pos_0[2*i+3])));
    }
    double u_t0 = -rho_i(pos_0, theta0) + (x0*sin(theta0) - y0*cos(theta0))*(sigma_func(psi_t)/pow(ri,2)) + s_i(pos_0,theta0,ki,gi)*sigma_func(psi_t) - (2/3) * gamma_i(x0,y0,theta0) * temp_sigma;
    double u_t = u_t0;
    double omega_t0 = -s_i(pos_0,theta0,ki,gi)*vi_t - sigma_func(psi_t);
    //end of initial conditions
    double omega_t = omega_t0;
    double beta_k[3];
    beta_k[0] = beta_t;
    double u_k[3];
    u_k[0]=u_t;
    double prev_v = vi_t;
    //control parameters
    unsigned long int k = 0; //discrete step

    geometry_msgs::Twist vn_1, vn_3; // neighbors' velocities
    //A1 = 3; A2 = 3; B1 = 3; B2 = 3;
    //A1 = 10; A2 = 3; B1 = 28; B2 = 3;
    //end of control parameters
    
    while (nh_.ok()){
        try{
            double t_now = ros::Time::now().toSec(); // integrate function to this time 
            
            listener_.lookupTransform("/irobot2", "/world", ros::Time(0), transform_);//listen to current frame
            listener1_.lookupTransform("/irobot1", "/world", ros::Time(0), transform1_);
            listener3_.lookupTransform("/irobot3", "/world", ros::Time(0), transform3_);
            listener1_.lookupTwist("/irobot1", "/world", ros::Time(0),ros::Duration(0.1) ,vn_1);
            listener3_.lookupTwist("/irobot3", "/world", ros::Time(0),ros::Duration(0.1), vn_3);

            double current_roll,current_pitch,current_yaw; //get current yaw
            transform_.getBasis().getRPY(current_roll,current_pitch,current_yaw); 
            double current_x, current_y, x1, y1, x3, y3; //get current positions
            current_x = transform_.getOrigin().x();
            current_y = transform_.getOrigin().y();
            x1 = transform1_.getOrigin().x();
            y1 = transform1_.getOrigin().y();
            x3 = transform3_.getOrigin().x();
            y3 = transform3_.getOrigin().y();
            
            double pos_t[6] = {current_x,current_y,x1,y1,x3,y3};
            vel_neighbor[0]=vn_1.linear.x;
            vel_neighbor[1]=vn_3.linear.x;
            //vel_neighbor[2] = {vel_irobot1,vel_irobot3};
            if (k>0){
                Vi_t = vi_t/Rw;
                Ri_t = vi_t/omega_t;
                vi_l = Vi_t*(1-L/(2*Ri_t));
                vi_r = Vi_t*(1+L/(2*Ri_t));
                vi_t = 0.5*Rw*(vi_l+vi_r);
                omega_t = Rw*(vi_l+vi_r)/L;
            }
            
            cout << "current v_t: " << vi_t << " current angular: " << omega_t <<"\n";
            geometry_msgs::Twist vel_;  
            vel_.linear.x = vi_t;
            vel_.angular.z = omega_t;
            cmd_vel_pub_.publish(vel_); //publish velocities

            k++; // next step 
            ki = 0.4/(1+pow(vi_t,2));
            gi = 0.3/(sqrt(1+pow(vi_t,2)));
            ri = r_i(current_x,current_y);
            beta_t = beta_i(current_x,current_y,current_yaw,vi_t); // beta at step k
            double temp_k0 = beta_k[0];
            double temp_k1 = beta_k[1];
            beta_k[0] = beta_t;
            beta_k[1] = temp_k0;
            beta_k[2] = temp_k1; 
            if (k==1)
            	var_theta_t = (h/2)*(beta_t0 + beta_t);
            else
            	var_theta_t = beta_k[1] + (h/3)*(beta_k[2] + 4*beta_k[1] + beta_k[0]);
            psi_t = current_yaw - var_theta_t - M_PI/2;

            u_t = u_tf(pos_t,vi_t,current_yaw,vel_neighbor,psi_t,ki,gi); // u_t at step k
            temp_k0 = u_k[0];
            temp_k1 = u_k[1];
            u_k[0] = u_t;
            u_k[1] = temp_k0;
            u_k[2] = temp_k1;

            if (k==1){
            	vi_t = (h/2)*(u_k[0] + u_k[1]);
            	prev_v = vi_t;
            }
            else{
            	vi_t = prev_v + (h/3)*(u_k[2]+4*u_k[1]+u_k[0]);
            	prev_v = vi_t;
            }
            omega_t = -s_i(pos_t,current_yaw,ki,gi)*vi_t - sigma_func(psi_t);
                      
        }
        catch(tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(0.5).sleep();
        }
        ros::spinOnce();
        rate.sleep();
    }
}