// Author: Benned Hedegaard

#include "executive/executive.h"

int main( int argc, char* argv[] ){

  // TODO - Move these up into a launch file via system arguments
	double reached_radius = 0.05; // Within 5cm is considered reached waypoint.
	double replan_radius = 0.5; // Within 0.5m we stop replanning.
	Executive exec( reached_radius, replan_radius );
	
	ros::init( argc, argv, "executive_node" );
	ros::NodeHandle node_handle;
	
	//Set up subscribers
	ros::Subscriber odom_sub = node_handle.subscribe( "simulator/odom", 1, &Executive::handleOdom, &exec );
	ros::Subscriber waypoint_sub = node_handle.subscribe( "executive/waypoint", 1, &Executive::handleWaypoint, &exec );
	
	// Set up any publishers inside the class instance.
	exec.query_pub = node_handle.advertise<planner::Query>( "planner/query", 1, true );

	sleep(1); // Wait 1 second; gives time for ROS connections to be made.
	
	while( ros::ok() ){
    ros::spinOnce(); // Calls all waiting callbacks, i.e. handles messages.
    sim.step( 1.0 / frequency ); // Advance the class' state belief.
    //call specific function from executive.cpp
    //set the timer to the current time
    timer.sleep(); // Waits rest of cycle to assure proper hz
  }


  }

  ros::spin();
	
	return 0;
}
