// Author: Benned Hedegaard

#include "planner/grid-planner.h"
#include "mapper/occmapper.h"

int main( int argc, char* argv[] ){
	
	double planner_discretization  = 0.1;
	
	double map_resolution = 0.1; // Meters per cell side
	int map_width = 101; // Number of cells
	int map_height = 101;
	
  // Center of each cell will align with planning grid nodes
  geometry_msgs::Pose origin;
	origin.position.x = -5.05;
	origin.position.y = -5.05;
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
	
  OccupancyGrid mapper( map_resolution, map_width, map_height, origin, obstacle_width, threshold, p0, p_free, p_occ );
  GridPlanner planner( planner_discretization, mapper ); // TODO - Obviously should only pass a CostMap
	
	ros::init( argc, argv, "planner_node" );
	ros::NodeHandle node_handle;
	
	ros::Subscriber query_sub = node_handle.subscribe( "planner/query", 1, &GridPlanner::handleQuery, &planner );
	ros::Subscriber map_sub = node_handle.subscribe( "mapper/map", 1, &GridPlanner::handleMap, &planner );
	
	// Set up any publishers inside the class instance.
	planner.path_pub = node_handle.advertise<planner::Path>( "planner/path", 1, true );
	planner.open_list_size_pub = node_handle.advertise<std_msgs::UInt32>( "planner/open_list_size", 1, true );
	planner.closed_list_size_pub = node_handle.advertise<std_msgs::UInt32>( "planner/closed_list_size", 1, true );
	
	sleep(1); // Wait 1 second; gives time for ROS connections to be made.
	
	ros::spin();
	
	return 0;
}
