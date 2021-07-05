// Author: Benned Hedegaard

#ifndef EXECUTIVE_H
#define EXECUTIVE_H

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include "planner/Query.h"

class Executive {

	public:
	
		/*
			Constructor for Exective class.
			reached - Distance (m) within which a waypoint is considered reached.
			replan - Distance (m) within which replanning is halted.
		*/
		Executive( const double& reached, const double& replan ); // Constructor
		virtual ~Executive(); // Deconstructor
		
		// Declare message handling functions for the class.
		void handleOdom( const nav_msgs::Odometry::ConstPtr& msg );
		void handleWaypoint( const geometry_msgs::Point::ConstPtr& msg );
		
		void sendQuery();
		
		// Declare any ROS publishers.
		ros::Publisher query_pub; // TODO - Move publishers out of all classes for more general code
		
	protected:
	
		bool hasOdom; // Have we stored an odometry yet?
		nav_msgs::Odometry _odom; // Store current state of robot.
		std::vector<geometry_msgs::Point> _waypoints; // Store current list of goals.
		
		double REACHED; // Within this distance counts as reaching a waypoint.
		double REPLAN; // Don't replan within this distance.
};

#endif /* EXECUTIVE_H */
