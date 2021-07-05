// Author: Benned Hedegaard

#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <Eigen/Dense>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "simulator/Obstacles.h"
#include "sensor_msgs/LaserScan.h"

class Simulator {

	public:
	
		Simulator(); // Constructor
		virtual ~Simulator(); // Deconstructor
		
		// Declare message handling functions for the class.
		void handleMotionCommand( const geometry_msgs::Twist::ConstPtr& msg );
		void handleObstacles( const simulator::Obstacles::ConstPtr& msg );
		
		void step( const double& dt );
		nav_msgs::Odometry getOdometry();
		sensor_msgs::LaserScan getScan( int beams );
		
	protected:
	    // TODO - Create a custom robot state and motion command type.
	
		// Also include static member variables. Start names with underscores.
		Eigen::Vector2d _u; // Current motion command.
		Eigen::Vector3d _x; // Current robot pose.
		simulator::Obstacles _obstacles; // Simulated obstacles.
};

#endif /* SIMULATOR_H */
