// This ROS node continually publishes a series of planning queries looping around a 3m-radius circle.
// Author: Benned Hedegaard

#include <cmath>

#include "ros/ros.h"
#include "planner/Query.h"
#include "geometry_msgs/Point.h"

int main( int argc, char* argv[] ){
	ros::init( argc, argv, "planner_demo_node" );
	ros::NodeHandle node_handle;
	
	// Set up any publishers inside the class instance.
	ros::Publisher query_pub = node_handle.advertise<planner::Query>( "planner/query", 1, true );
	
	sleep(1); // Wait 1 second; gives time for ROS connections to be made.
	
	geometry_msgs::Point start;
	start.x = 0.0;
	start.y = 0.0;
	double r = 3.0;
	double theta = 0.0;
	
	geometry_msgs::Point goal;
		
	double frequency = 5.0; // Desired rate of the timer in hz
	ros::Rate timer( frequency );
	while( ros::ok() ){
		goal.x = r*std::cos(theta);
		goal.y = r*std::sin(theta);
		
		planner::Query q;
		q.start = start;
		q.goal = goal;
		query_pub.publish(q);
		
		timer.sleep();
		theta += 0.05;
	}
	
	return 0;
}
