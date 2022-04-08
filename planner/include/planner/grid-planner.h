// Author: Benned Hedegaard

#pragma once

#include <memory>

#include "planner/node.h"

#include <vector>
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "planner/Path.h"
#include "planner/Query.h"
#include "std_msgs/UInt32.h"
#include "mapper/occupancygrid.h"

class GridPlanner{
public:

  GridPlanner( const double& discretization, const OccupancyGrid& mapArg );
  virtual ~GridPlanner() = default; // Default deconstructor

  // Declare message handling functions for the class.
  void handleQuery( const planner::Query::ConstPtr& msg );
  void handleMap( const nav_msgs::OccupancyGrid::ConstPtr& msg );
		
  // TODO - Move ROS publishers elsewhere
  ros::Publisher path_pub;
  ros::Publisher open_list_size_pub;
  ros::Publisher closed_list_size_pub;
		
protected:
	
  std::vector<geometry_msgs::Point> aStar( const geometry_msgs::Point& start, const geometry_msgs::Point& goal );
  OccupancyGrid cost_map; // TODO - Obviously the wrong datatype for this! Create CostMap in mapper package // Roy: fucntion wise, since occupied() method is used, changed datatype to the new OccupancyGrid instead of CostMap
		
  double DISCRETIZATION;
};
