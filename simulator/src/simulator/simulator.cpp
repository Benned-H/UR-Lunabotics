// Author: Benned Hedegaard

#include <cmath>

#include "geometry_msgs/Quaternion.h"

#include "simulator/simulator.h"

/*
	u(0) - v (linear velocity forwards)
	u(1) - w (angular velocity)
	
	x(0) - x position
	x(1) - y position
	x(2) - theta (heading)
*/
Simulator::Simulator() {
	_u(0) = 0.0;
	_u(1) = 0.0;
	
	_x(0) = 0.0;
	_x(1) = 0.0;
	_x(2) = 0.0;
}

Simulator::~Simulator() {} // Deconstructor

void Simulator::handleMotionCommand( const geometry_msgs::Twist::ConstPtr& msg ) {
	_u(0) = msg->linear.x; // vx
	_u(1) = msg->angular.z; // wz
}

void Simulator::handleObstacles( const simulator::Obstacles::ConstPtr& msg ) {
	_obstacles = *msg;
}

// TODO - Move to common package
geometry_msgs::Quaternion yaw_to_quat( double yaw ) {
	geometry_msgs::Quaternion q;
	q.w = cos(yaw/2.0);
	q.x = 0.0;
	q.y = 0.0;
	q.z = sin(yaw/2.0);
	
	return q;
}

nav_msgs::Odometry Simulator::getOdometry() {
	nav_msgs::Odometry msg;
	msg.header.stamp = ros::Time::now();
	msg.pose.pose.position.x = _x(0);
	msg.pose.pose.position.y = _x(1);
	msg.pose.pose.position.z = 0.0;
	msg.pose.pose.orientation = yaw_to_quat(_x(2));
	msg.twist.twist.linear.x = _u(0);
	msg.twist.twist.angular.z = _u(1);
	
	return msg;
}

// Input is how many laser beams should be simulated, spread around the scan.
// TODO - Test if this implementation returns correct number of beams
sensor_msgs::LaserScan Simulator::getScan( int beams ) {
	sensor_msgs::LaserScan scan;
	
	if (beams < 4) { // Force minimum of 4 laser beams.
		beams = 4;
	}
	
	scan.angle_increment = 2.0*M_PI / ( (double) beams );
	scan.angle_min = -M_PI; // Start at -Pi and increment to Pi.
	scan.angle_max = scan.angle_min + beams*scan.angle_increment;
	scan.range_min = 0.0;
	scan.range_max = 10.0;
	
	double x1 = _x(0);
	double y1 = _x(1);
	
	for ( int i = 0; i < beams; i++ ) {
		double relative_angle = scan.angle_min + i*scan.angle_increment;
		double global_angle = relative_angle + _x(2);
		if ( _obstacles.data.size() == 0 ) {
			scan.ranges.push_back(scan.range_max);
		} else {
			double closest = scan.range_max;
			double x2 = x1 + scan.range_max*cos(global_angle); // End of scan is (x2, y2)
			double y2 = y1 + scan.range_max*sin(global_angle);
			double a = (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2);
			
			for ( int o = 0; o < _obstacles.data.size(); o++ ) {
				// Notation is only to match my pure pursuit calculations.
				double xr = _obstacles.data[o].x;
				double yr = _obstacles.data[o].y;
				double l = _obstacles.data[o].z; // Radius of the obstacle
				
				double b = 2.0*(x1*x2 - x1*x1 + x1*xr - x2*xr + y1*y2 - y1*y1 + y1*yr - y2*yr);
				double c = x1*x1 + xr*xr + y1*y1 + yr*yr - l*l - 2.0*x1*xr - 2.0*y1*yr;
					
				double s = b*b - 4.0*a*c;
				if ( s < 0.0 ) {
					continue;
				}
					
				double t1 = (sqrt(s) - b) / (2.0*a);
				double t2 = (-sqrt(s) - b) / (2.0*a);
				
				if ( 0.0 < t1 && t1 < 1.0 ) { // Is t1 in the laser beam?
					double xt1 = x1 + t1*(x2-x1);
					double yt1 = y1 + t1*(y2-y1);
					double distance = sqrt( (xt1-x1)*(xt1-x1) + (yt1-y1)*(yt1-y1) );
					if ( distance < closest ) {
						closest = distance;
					}
				}

				if ( 0.0 < t2 && t2 < 1.0 ) { // Is t2 in the laser beam?
					double xt2 = x1 + t2*(x2-x1);
					double yt2 = y1 + t2*(y2-y1);
					double distance = sqrt( (xt2-x1)*(xt2-x1) + (yt2-y1)*(yt2-y1) );
					if ( distance < closest ) {
						closest = distance;
					}
				}
			}
			scan.ranges.push_back(closest);
		}
	}
	
	scan.header.stamp = ros::Time();
	scan.header.frame_id = "robot";
	
	return scan;
}

// From Probabilistic Robotics Version 1 pg. 127.
void Simulator::step( const double& dt ) {
	double v = _u(0);
	double w = _u(1);
	if ( w == 0.0 ) { // Avoid dividing by 0.
		w = 0.00001;
	}
	
	double new_x = _x(0) - (v/w)*std::sin(_x(2)) + (v/w)*std::sin(_x(2)+w*dt);
	double new_y = _x(1) + (v/w)*std::cos(_x(2)) - (v/w)*std::cos(_x(2)+w*dt);
	double new_theta = _x(2) + w*dt;
	
	_x(0) = new_x;
	_x(1) = new_y;
	_x(2) = new_theta;
}

