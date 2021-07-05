// Author: Benned Hedegaard

#ifndef OCC_MAPPER_H
#define OCC_MAPPER_H

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/LaserScan.h"

class OccMapper {

    // TODO - Create separate OccupancyGrid class to clean up which functions are where
    // TODO - Rename this class to something clearer

	public:
	
		/*
			OccMapper class constructor
			
			resolutionArg - Resolution (m) of the cost map
			mapWidthArg - Width (number of cells) of the cost map
			mapHeightArg - Height (number of cells) of the cost map
			origin - Origin of the cost map
			obstacleWidthArg - Presumed width (m) of a detected obstacle
			thresholdArg - Threshold of occupancy over which a cell is deemed occupied
			p0, p_free, p_occ - Probability parameters of the occupancy grid mapping algorithm
		*/
		OccMapper( const double& resolutionArg, const unsigned int& mapWidthArg, const unsigned int& mapHeightArg, const geometry_msgs::Pose& origin, const double& obstacleWidthArg, const int& thresholdArg, const double& p0, const double& p_free, const double& p_occ );
		virtual ~OccMapper(); // Deconstructor
		
		// Declare message handling functions for the class.
		void handleOdom( const nav_msgs::Odometry::ConstPtr& msg );
		void handleLaserscan( const sensor_msgs::LaserScan::ConstPtr& msg );
		void handleClick( const geometry_msgs::PointStamped::ConstPtr& msg );
		
		void publishMap();
		bool occupied( const double& x, const double& y );
		
		// Declare any ROS publishers. TODO - Move outside class
		ros::Publisher map_pub;
		
		nav_msgs::OccupancyGrid _map;
		
	protected:
	
		int x_to_col( const double& x );
		int y_to_row( const double& y );
		int point_to_index( const double& x, const double& y );
		bool inMap( const double& x, const double& y );
		void updateMap( const geometry_msgs::Pose& pose, const sensor_msgs::LaserScan& scan );
		
		geometry_msgs::Pose _pose;
		sensor_msgs::LaserScan _scan;
		int _occ_steps; // Depth of obstacles in steps of size RESOLUTION.
		int _threshold; // We consider cells with this value or greater occupied.
		
		double l0; // Prior log-odds of occupancy.
		double l_occ; // Log-odds of occupied cell given laser hit.
		double l_free; // Log-odds of free cell given laser passes through.
		
		// Map parameters
		double RESOLUTION;
		double MIN_X;
		double MAX_X;
		double MIN_Y;
		double MAX_Y;
};

#endif /* OCC_MAPPER_H */
