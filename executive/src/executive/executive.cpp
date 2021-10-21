/**
 * Implements the executive function for our autonomy architecture by querying the planner node
 * Author: Benned Hedegaard
 */

#include <cmath>
#include "executive/executive.h"

//
// Parameter constructor for Exective class
//
Executive::Executive( const double reachedArg, const double replanArg ) :
    state(), waypoints(), reached_distance( reachedArg ), replan_distance( replanArg ){}
//
// Message handler for nav_msgs::Odometry messages
//
void Executive::handleOdom( const nav_msgs::Odometry::ConstPtr& msg ){
	state = std::optional< nav_msgs::Odometry >( *msg ); // Store the message
	
	// Exit if there are no more waypoints
	if( waypoints.empty() ) return;
		
	// Check if we've reached the current waypoint. If so, remove it
	double xr = state->pose.pose.position.x;
	double yr = state->pose.pose.position.y;
	geometry_msgs::Point curr_waypoint = waypoints.front();
	
	if( euclidean( xr, yr, curr_waypoint.x, curr_waypoint.y ) < reached_distance ){
		waypoints.erase( waypoints.begin() );
		// Exit if there are no more waypoints.
		if( waypoints.empty() ) return;
		curr_waypoint = waypoints.front();
	}
	
	// If we're close to the current waypoint, don't create a new plan
	if( euclidean( xr, yr, curr_waypoint.x, curr_waypoint.y ) < replan_distance ) return;
	
	sendQuery(); // Otherwise create a new plan by querying the planner
  return;
}

//
// Message handler for geometry_msgs::Point waypoint messages
//
void Executive::handleWaypoint( const geometry_msgs::Point::ConstPtr& msg ){
	waypoints.push_back( *msg );
	
	if( state && ( waypoints.size() == 1 ) ){
		sendQuery(); // New waypoint is our first
	}
}
// CONTINUE
// This function will always send a new query based on the first waypoint.
void Executive::sendQuery(){	
	planner::Query q;
	q.start = state.pose.pose.position;
	q.goal = waypoints.front();
	
	query_pub.publish(q);
}
