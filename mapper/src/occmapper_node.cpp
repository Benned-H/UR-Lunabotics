// Author: Benned Hedegaard

#include "mapper/occmapper.h"

int main( int argc, char* argv[] ){
  // TODO - Move arguments up to launch file using commandline args
	double map_resolution = 0.1; // Meters per cell side
	int map_width = 101; // Number of cells
	int map_height = 101;
	
    // Center of each cell will align with planning grid nodes
	geometry_msgs::Pose origin;
	origin.position.x = -6.89;
	origin.position.y = -1.25;
	origin.position.z = 0.0;
	origin.orientation.x = 0.0;
	origin.orientation.y = 0.0;
	origin.orientation.z = 0.0;
	origin.orientation.w = 1.0;

	double obstacle_width = 0.2; // Presumed width of an obstacle
	double threshold = 0.3; // Above this value, cell is assumed occupied
	
	double p0 = 0.3; // Prior probability a cell is occupied
	double p_free = 0.05; // Probability a cell is occupied given laser passes through it
	double p_occ = 0.95; // Probability a cell is occupied given laser hits in it
	
	OccMapper occ( map_resolution, map_width, map_height, origin, obstacle_width, threshold, p0, p_free, p_occ );
	
	ros::init( argc, argv, "occmapper_node" );
	ros::NodeHandle node_handle;

	// Set up any subscribers
	ros::Subscriber odom_sub = node_handle.subscribe( "simulator/odom", 1, &OccMapper::handleOdom, &occ );
	ros::Subscriber laserscan_sub = node_handle.subscribe( "scan", 3, &OccMapper::handleLaserscan, &occ ); // Stores up to 3 laserscans.
	/*ros::Subscriber click_sub = node_handle.subscribe( "clicked_point", 1, &OccMapper::handleClick, &occ ); // Helps debug grid locations. */
	
	// Set up any publishers inside the class instance.
	occ.map_pub = node_handle.advertise<nav_msgs::OccupancyGrid>( "mapper/map", 1, true );
	occ.publishMap();

	sleep(1); // Wait 1 second; gives time for ROS connections to be made.
			
	ros::spin();
	
	return 0;
}
