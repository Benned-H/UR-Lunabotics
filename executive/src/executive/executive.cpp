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

//Create an enum to track states.
enum State{Initialize, ManualControl, Localize, GoToMine, SetupMine, Mine, SetupDeposit, Deposit, Terminate};

// TODO - Move to a common package instead
double euclidean( double x1, double y1, double x2, double y2 ){
	return std::sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) );
}

void Executive::stateMachine(){
	State curr = Initialize;
	bool run = true;
	while(run){
		switch(curr){
			case Initialize:
				//returns Localize, ManualControl, or Terminate
				curr = Initialize();	
				break;
			case ManualControl:
				//code here
				break;
			case Localize:
				//code here
				break;
			case GoToMine:
				//code here
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
