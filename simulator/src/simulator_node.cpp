// Author: Benned Hedegaard

#include "ros/ros.h"
#include "simulator/simulator.h"

int main( int argc, char* argv[] ){
	Simulator sim;
	Simulator nsim;
	ros::init( argc, argv, "simulator_node" );
	ros::NodeHandle node_handle;
	
	ros::Subscriber command_sub = node_handle.subscribe( "motion_commands", 1, &Simulator::handleMotionCommand, &sim );
	ros::Subscriber obstacles_sub = node_handle.subscribe( "simulator/obstacles", 1, &Simulator::handleObstacles, &sim );
	
	ros::Subscriber ncommand_sub = node_handle.subscribe( "motion_commands", 1, &Simulator::handleMotionCommand, &nsim );
	ros::Subscriber nobstacles_sub = node_handle.subscribe( "simulator/obstacles", 1, &Simulator::handleObstacles, &nsim );
	
	ros::Publisher odom_pub = node_handle.advertise<nav_msgs::Odometry>( "simulator/odom", 1, true );
	ros::Publisher nodom_pub = node_handle.advertise<nav_msgs::Odometry>( "simulator/nodom", 1, true );
	ros::Publisher scan_pub = node_handle.advertise<sensor_msgs::LaserScan>( "scan", 1, true );
		
	sleep(1); // Wait 1 second; gives time for ROS connections to be made.
		
	double frequency = 20.0; // Desired rate of the timer in hz
	ros::Rate timer( frequency );
	
	// Keeps looping as long as the node is running.
	while( ros::ok() ){
		ros::spinOnce(); // Calls all waiting callbacks, i.e. handles messages.
		sim.step( 1.0 / frequency , true); // Advance the class' simulation/belief.
		nsim.step( 1.0 / frequency , false); // Advance the class' simulation/belief.
		//sim and nsim are inverted. sim is the noisy data rn to change back set the bool to false
		//simnoisy.step(1.0/frequency,true);
		odom_pub.publish( sim.getOdometry() );
		nodom_pub.publish( nsim.getOdometry() ); //noisy
		scan_pub.publish( sim.getScan(360) ); // Number here = # laser beams.
		timer.sleep(); // Waits rest of cycle to assure proper hz
	}
	
	return 0;
}
