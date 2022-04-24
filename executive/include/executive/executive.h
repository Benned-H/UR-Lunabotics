// Author: Benned Hedegaard

#pragma once

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include "planner/Query.h"

class Executive {
public:

//Create an enum to track states. 
enum State{Initialize, ManualControl, Localize, GoToMine, SetupMine, Mine, SetupDeposit, Deposit, Terminate}; 

  /*
    Constructor for Exective class.
		reached - Distance (m) within which a waypoint is considered reached.
		replan - Distance (m) within which replanning is halted.
	*/
	Executive( const double& reachedArg, const double& replanArg ); // Constructor
	
  virtual ~Executive() = default; // Default deconstructor
		
  // Declare message handling functions for the class.
	void handleOdom( const nav_msgs::Odometry::ConstPtr& msg );
	void handleWaypoint( const geometry_msgs::Point::ConstPtr& msg );
  
  /*
    TODO - Document this method
  */
  void sendQuery();
	
  ros::Publisher query_pub; // TODO - Move publishers out of all classes for more general code
		
protected:
  bool hasOdom; // Have we stored an odometry yet?
	nav_msgs::Odometry state; // Store current state of robot.
	std::vector<geometry_msgs::Point> waypoints; // Store current list of goals.

  double reached_distance; // Within this distance counts as reaching a waypoint.
  double replan_distance; // Don't replan within this distance.

  /* Current state of the state machine */
  State curr;

  //make variables for duration for each state

private:
};
