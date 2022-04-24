// Author: Benned Hedegaard

#include <cmath>
#include "executive/executive.h"

/*
	Constructor for Exective class.
	reached - Distance (m) within which a waypoint is considered reached.
	replan - Distance (m) within which replanning is halted.
*/
Executive::Executive( const double& reachedArg, const double& replanArg ) :
    reached_distance( reachedArg ), replan_distance( replanArg ), hasOdom( false ){}


	State curr = Initialize;

// TODO - Move to a common package instead
double euclidean( double x1, double y1, double x2, double y2 ){
	return std::sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) );
}

// Steps the state machine forward
void Executive::step(){
	switch(curr){
			case Initialize:
				//returns Localize, ManualControl, or Terminate
				curr = Initialize();	
				break;
			case ManualControl:
				//returns Terminate
				curr = ManualControl();
				break;
			case Localize:
				//returns ManualControl or GoToMine
				curr = Localize();
				break;
			case GoToMine:
				curr = GoToMine();
				break;
			case SetupMine:
				//code here
				break;
			case Mine:
				//code here
				break;
			case SetupMine:
				//code here
        break;
			case Mine:
				//code here
        break;
			case SetupDeposit:
				//code here
                	        break;
			case Deposit:
				//code here
                	        break;
			case Terminate:
				//code here
				run = false;
                	        break;
		}
	}

}

State Executive::Initialize(){
	//bool are motors connected?
	//bool is camera connected?
	//bool is telemetry signal working?
	// if motors connected, camera connected
		//if telemetry works return Localize
		//else try signal again, if fails return Terminate
	//else if motors connected
		//check telemetry - if works then return ManualControl
}

State Executive::Localize(){
	//bool rotate = true;
	//while (rotate OR rotated for x seconds?)
		//rotate?
		//build costmap???
		//if (signal for target found), return GoToMine;
		//if (rotated for x seconds), return ManualControl;
	//			
//To find the rotation rate of the robot, turn another 360º, count until we locate the target beacon again, then use this during the rest of planning.
}

State Executive::GoToMine(){
	//send planning query to nearest point in mining zone
	//target obstacle free and unmined mining lane
	//localization continuously integrates obstacle information into costmap, actions generated to move towards waypoint
	//if old waypoint is on high cost region, consider sending new planning query
	//if free space or pose estimate is in mining zone, return SetupMine	

}

State Executive::SetupMine(){
	
	//turn around 180 deg and locate target beacon
		// rotate until target beacon found, insert some timeout 
	//once located check distnace x >4.39 m from target (add cushioning for accuracy?



[Instance] The robot will turn around and locate the target beacon.
[If, target beacon is located]
The robot will reduce uncertainty about its distance far from the collector bin.
The front end of the robot must have crossed into the Mining Arena, which begins 4.39 m from the starting zone.
[If, robot x < 4.39 m] Move into the Mining Zone.
Transition to the Mine state once the robot is properly within the Mining Zone and in an unmined lane (based on previously mined lanes).
	

}





void Executive::handleOdom( const nav_msgs::Odometry::ConstPtr& msg ){
	state = *msg;
	hasOdom = true;
	
	// Exit if there are no more waypoints.
	if ( waypoints.size() == 0 ) {
		return;
	}
		
	// Check if we've reached the current waypoint. If so, remove it.
	double xr = state.pose.pose.position.x;
	double yr = state.pose.pose.position.y;
	geometry_msgs::Point curr_waypoint = waypoints.front();
	
	if( euclidean( xr, yr, curr_waypoint.x, curr_waypoint.y ) < reached_distance ){
		waypoints.erase( waypoints.begin() );
		// Exit if there are no more waypoints.
		if( waypoints.size() == 0 ){
			return;
		}
		curr_waypoint = waypoints.front();
	}
	
	// If we're close to the current waypoint, don't create a new plan.
	if( euclidean( xr, yr, curr_waypoint.x, curr_waypoint.y ) < replan_distance ){
		return;
	}
	
	sendQuery();
}

void Executive::handleWaypoint( const geometry_msgs::Point::ConstPtr& msg ) {
	waypoints.push_back( *msg );
	
	if( hasOdom && ( waypoints.size() == 1 ) ){
		sendQuery(); // New waypoint is our first.
	}
}

// This function will always send a new query based on the first waypoint.
void Executive::sendQuery(){	
	planner::Query q;
	q.start = state.pose.pose.position;
	q.goal = waypoints.front();
	
	query_pub.publish(q);
}
