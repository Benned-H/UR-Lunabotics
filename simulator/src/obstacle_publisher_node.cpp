// Publishes a specified number of simulated cylindrical obstacles.
// Author: Benned Hedegaard

#include <stdlib.h>

#include "ros/ros.h"
#include "simulator/Obstacles.h"
#include "geometry_msgs/Point.h"

int main( int argc, char* argv[] ) {
	ros::init( argc, argv, "obstacle_publisher_node" );
	ros::NodeHandle node_handle;
	
	ros::Publisher obstacles_pub = node_handle.advertise<simulator::Obstacles>( "simulator/obstacles", 1, true );
	
	srand(time(NULL)); // Initialize random seed using current time.
	sleep(1); // Wait 1 second; gives time for ROS connections to be made.
	
	simulator::Obstacles msg;
	
	// TODO - Move arguments to a rosparams file instead
	// TODO - Generating random obstacles via Poisson random forest?
	int NUMBER_OF_OBSTACLES = 6;
	int successes = 0;
	while ( successes < NUMBER_OF_OBSTACLES ) {
		geometry_msgs::Point p;
		
		// TODO - Move RNG to common function package
		p.x = (rand() % 585)*0.01 + 0.50; // Between 0.50 and 6.34m
		p.y = (rand() % 151)*0.01 - 0.75; // Between -0.75 and 0.75m
		int rng = rand() % 21; // RNG 0 to 20
		p.z = 0.30 + rng*0.001; // Will be 0.30 to 0.5. Radius of the obstacle.
		
		// Too close to the origin.
		if ( sqrt(p.x*p.x + p.y*p.y) - p.z < 0.5 ) {
			continue;
		} else {
			bool skip = false;
			for ( int i = 0; i < msg.data.size(); i++ ) { // Check unique.
				if ( msg.data[i].x == p.x && msg.data[i].y == p.y ) {
					skip = true;
					break;
				}
			}
			
			if (skip) continue;
			successes++;
			msg.data.push_back(p);
		}
	}
	
	obstacles_pub.publish(msg);
	
	ros::spin();
	
	return 0;
}
