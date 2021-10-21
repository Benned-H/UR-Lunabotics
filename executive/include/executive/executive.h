/**
 * Implements the executive function for our autonomy architecture by querying the planner node
 * Author: Benned Hedegaard
 */

#pragma once

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include "planner/Query.h"

class Executive {
public:

  /**
    Parameter constructor for Exective class

    @brief Parameter constructor for Exective class
    @param[in]    reachedArg    distance (m) within which a waypoint is considered reached
		@param[in]    replanArg     distance (m) within which replanning is halted
    @returns      none
	*/
	Executive( const double reachedArg, const double replanArg );
	
  /**
    Default deconstructor for Executive class
  */
  virtual ~Executive() = default;
		
	/**
    Message handler for nav_msgs::Odometry messages

    @brief Message handler for nav_msgs::Odometry messages
    @param[in]  msg   updated odometry providing the robot's pose
    @returns    none
  */
  void handleOdom( const nav_msgs::Odometry::ConstPtr& msg );
	
  /**
    Message handler for geometry_msgs::Point waypoint messages

    @brief Message handler for geometry_msgs::Point waypoint messages
    @param[in]  msg   next waypoint for the robot to move toward
    @returns    none
  */
  void handleWaypoint( const geometry_msgs::Point::ConstPtr& msg );
  
  /**
    Method to send a query to the planner

    @brief Method to send a query to the planner
    @returns    none
  */
  void sendQuery();

  /** ros::Publisher to publish queries to the planner */
  ros::Publisher query_pub;
		
protected:
  /** Stored current state of the robot. Optional to indicate when we've received this */
	std::optional< nav_msgs::Odometry > state;

  /** Current list of goals to move to next */
	std::vector<geometry_msgs::Point> waypoints;

  /** Within this distance (m) counts as reaching a waypoint */
  double reached_distance;

  /** Don't replan within this distance (m) */
  double replan_distance;

private:
};
