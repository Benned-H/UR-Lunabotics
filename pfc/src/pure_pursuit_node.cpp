// Author: Benned Hedegaard

#include "pfc/pure-pursuit.h"

int main( int argc, char* argv[] ){
  // TODO - Move arguments to an external launch file
	double lookahead_distance = 0.5; // Meters
	double turn_angle = M_PI/4.0; // Beyond this heading difference, will turn-in-place.
	double default_vx = 0.5; // Meters per second
	double default_wz = 0.6; // Radians per second

	PurePursuit pfc( lookahead_distance, turn_angle, default_vx, default_wz );
	
	ros::init( argc, argv, "pure_pursuit_node" );
	ros::NodeHandle node_handle;
	
	ros::Subscriber odom_sub = node_handle.subscribe( "simulator/odom", 1, &PurePursuit::handleOdom, &pfc );
	ros::Subscriber path_sub = node_handle.subscribe( "planner/path", 1, &PurePursuit::handlePath, &pfc );
	
	// Set up any publishers inside the class instance.
	pfc.command_pub = node_handle.advertise<geometry_msgs::Twist>( "motion_commands", 1, true );
	pfc.goal_point_pub = node_handle.advertise<geometry_msgs::Point>( "pfc/goal_point", 1, true );
		
	sleep(1); // Wait 1 second; gives time for ROS connections to be made.
	
	ros::spin();
	
	return 0;
}
