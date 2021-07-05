// Author: Benned Hedegaard

#include <cmath>
#include <set>
#include <iostream>

#include "mapper/occmapper.h"

// Formula from Wikipedia for now. More quaternion understanding is needed.
// TODO - Move to common
double quat_to_yaw( const geometry_msgs::Quaternion& q ) {
	return atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z));
}

// Returns log-odds representation of the given probability.
double log_odds( const double& probability ) {
	return std::log( probability / (1.0 - probability) );
}


/*
	OccMapper class constructor

	resolutionArg - Resolution (m) of the cost map
	mapWidthArg - Width (number of cells) of the cost map
	mapHeightArg - Height (number of cells) of the cost map
	origin - Origin of the cost map
	obstacleWidthArg - Presumed width (m) of a detected obstacle
	thresholdArg - Threshold of occupancy over which a cell is deemed occupied
	p0, p_free, p_occ - Prior, free, and occupied probabilities of cells given laser hit
*/
OccMapper::OccMapper( const double& resolutionArg, const unsigned int& mapWidthArg, const unsigned int& mapHeightArg, const geometry_msgs::Pose& origin, const double& obstacleWidthArg, const int& thresholdArg, const double& p0, const double& p_free, const double& p_occ ) {
	RESOLUTION = resolutionArg;
	if ( RESOLUTION == 0.0 )
		RESOLUTION = 0.01;
	_map.info.resolution = resolutionArg;
	_map.info.width = mapWidthArg;
	_map.info.height = mapHeightArg;
	_map.info.origin = origin;
	
	std::vector<int8_t> data( mapWidthArg*mapHeightArg, 0 );
	_map.data = data;
	
	// (0,0) in the map is its bottom-left corner in the 2D plane.
	MIN_X = origin.position.x;
	MAX_X = origin.position.x + mapWidthArg*RESOLUTION;
	MIN_Y = origin.position.y;
	MAX_Y = origin.position.y + mapHeightArg*RESOLUTION;
	
	// How many steps of size resolution are obstacles deep?
	_occ_steps = std::floor( obstacleWidthArg/RESOLUTION);
	_threshold = thresholdArg;
	
	// Input appropriate p0, p_free, and p_occ.
	l0 = log_odds(p0);
	l_free = log_odds(p_free);
	l_occ = log_odds(p_occ);
}

OccMapper::~OccMapper() {} // Deconstructor

void OccMapper::handleOdom( const nav_msgs::Odometry::ConstPtr& msg ) {
	_pose = msg->pose.pose;
}

void OccMapper::handleLaserscan( const sensor_msgs::LaserScan::ConstPtr& msg ) {
	_scan = *msg;
	updateMap( _pose, _scan );
}

void OccMapper::handleClick( const geometry_msgs::PointStamped::ConstPtr& msg ) {
	std::cout << "Handling point..." << std::endl;
	int index = point_to_index( msg->point.x, msg->point.y );
	std::cout << "Index was " << index << "." << std::endl;
	_map.data[index] = 100;
	publishMap();
	std::cout << "New map published." << std::endl;
}

void OccMapper::publishMap() {
	_map.header.stamp = ros::Time::now();
	map_pub.publish(_map);
}

// Returns if a given cell is occupied.
bool OccMapper::occupied( const double& x, const double& y ) {
	if ( inMap(x, y) ) {
		int index = point_to_index( x, y );
		return ( _map.data[index] > _threshold );
	} else {
		return true; // TODO - How do we handle out-of-map queries?
	}
}

// Returns if the given (x,y) point is in the map's range.
bool OccMapper::inMap( const double& x, const double& y ) {
	return ((MIN_X < x) && (x < MAX_X) && (MIN_Y < y) && (y < MAX_Y));
}

// Returns the map's column index for the given x coordinate.
int OccMapper::x_to_col( const double& x ) {
	return floor((x - _map.info.origin.position.x) / RESOLUTION);
}

// Returns the map's row index for the given y coordinate.
int OccMapper::y_to_row( const double& y ) {
	return floor((y - _map.info.origin.position.y) / RESOLUTION);
}

// Returns the index in the grid of the given (x,y) point.
int OccMapper::point_to_index( const double& x, const double& y ) {
	return _map.info.width * y_to_row(y) + x_to_col(x);
}

void OccMapper::updateMap( const geometry_msgs::Pose& pose, const sensor_msgs::LaserScan& scan ) {
    if ( scan.ranges.size() == 0 ) {
        return; // No new information because scan was empty.
	}
	
	double heading = quat_to_yaw( pose.orientation );
	
	// Track cells that need to be updated from each laser.
	std::set<int> occupied_cells;
	std::set<int> free_cells;
	
	for ( int i = 0; i < scan.ranges.size(); i++ ) {
		// Clear sets for each laser beam; update cells at most once per laser.
		occupied_cells.clear();
		free_cells.clear();
		
		if ( (scan.ranges[i] < scan.range_min) || (scan.ranges[i] > scan.range_max) ) {
			continue; // Skip invalid laser returns.
		}
		
		double bearing = scan.angle_min + i*scan.angle_increment; // radians
		double global_bearing = heading + bearing; // No need to normalize; only passed into cos/sin.
		
		int free_steps = floor( scan.ranges[i] / RESOLUTION ); // Number of steps we need to take along the beam
		
		// Step along the laser scan; all cells here are free.
		for ( int step = 0; step < free_steps; step++ ) {
			double range = step*RESOLUTION;
			double x = pose.position.x + range*std::cos(global_bearing);
			double y = pose.position.y + range*std::sin(global_bearing);
			
			// Avoid accessing out-of-bounds cells.
			if ( !inMap(x, y) ) {
				continue;
			}
			
			int cell_index = point_to_index(x, y);
			free_cells.insert(cell_index);
		}
		
		// Step across the occupied range for this laser.
		for ( int step = 0; step < _occ_steps; step++ ) {
			double range = scan.ranges[i] + step*RESOLUTION;
			double x = pose.position.x + range*std::cos(global_bearing);
			double y = pose.position.y + range*std::sin(global_bearing);
			
			// Avoid accessing out-of-bounds cells.
			if ( !inMap(x, y) ) {
				continue;
			}
			
			int cell_index = point_to_index(x, y);
			occupied_cells.insert(cell_index);
		}
		
		// Now update each grid cell accordingly.
		for ( std::set<int>::iterator free = free_cells.begin(); free != free_cells.end(); ++free ) {
			double result = _map.data[*free] + l_free - l0;
			if ( result < 0.0 ) {
				result = 0;
			} else if ( result > 100.0 ) {
				result = 100;
			}
			_map.data[*free] = (int) result;
		}
		
		for ( std::set<int>::iterator occ = occupied_cells.begin(); occ != occupied_cells.end(); ++occ ) {
			double result = _map.data[*occ] + l_occ - l0;
			if ( result < 0.0 ) {
				result = 0;
			} else if ( result > 100.0 ) {
				result = 100;
			}
			_map.data[*occ] = (int) result;
		}
	}
	
	publishMap();
}
