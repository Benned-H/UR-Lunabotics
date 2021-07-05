// Author: Benned Hedegaard

#include "pfc/purepursuit.h"

PurePursuit::PurePursuit( const double& lookahead, const double& turn_angle, const double& forward_v, const double& turning_w ) {
	LOOKAHEAD = lookahead;
	TURNING_ANGLE = turn_angle;
	DEFAULT_V = fabs(forward_v); // Ensure these are positive just in case
	DEFAULT_W = fabs(turning_w);
	hasOdom = false;
	hasPath = false;
}

PurePursuit::~PurePursuit() {} // Deconstructor

void PurePursuit::handleOdom( const nav_msgs::Odometry::ConstPtr& msg ) {
	_odom = *msg;
	hasOdom = true;
	if (hasPath) {
		purePursuit();
	}
}

void PurePursuit::handlePath( const planner::Path::ConstPtr& msg ) {
	if ( (msg->points).size() == 0 ) {
		return; // Ignore empty plans
	}
	
	_path = *msg;
	hasPath = true;
	
	if (hasOdom) {
		purePursuit();
	}
}

// Formula from Wikipedia for now. More quaternion understanding is needed.
// TODO - Move this to a common package, understand its derivation
double PurePursuit::quat_to_yaw( const geometry_msgs::Quaternion& q ) {
	return atan2(2.0*(q.w*q.z + q.x*q.y), q.w*q.w+q.x*q.x-q.y*q.y-q.z*q.z);
}

// TODO - Move this to a common package, pass by const reference
double euclidean( double x1, double y1, double x2, double y2 ) {
	return sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) );
}

// Compute the goal point for the given robot pose and planned path.
// Returns the robot pose if the search fails.
geometry_msgs::Point PurePursuit::getGoalPoint( const geometry_msgs::Pose& pose, const planner::Path& path ) {
	// 1. Find current location of the robot.
	double xr = pose.position.x;
	double yr = pose.position.y;
	
	// 2. Find the path point closest to the robot. We search from there.
	int closest_path_index = 0;
	double min_dist = euclidean( xr, yr, path.points[0].x, path.points[0].y );
	
	for (int i = 1; i < path.points.size(); i++) {
		double dist_i = euclidean( xr, yr, path.points[i].x, path.points[i].y );
		if (dist_i < min_dist) {
			closest_path_index = i;
			min_dist = dist_i;
		}
	}
	
	// If the closest path point is the last, just return that.
	geometry_msgs::Point last_point =  path.points.back();
	int last_index = path.points.size() - 1;
	if ( path.points[closest_path_index] == last_point ) {
		return last_point;
	}
	
	// Also target the final path point if we're close enough.
	if ( euclidean(xr, yr, last_point.x, last_point.y) < LOOKAHEAD ) {
		return last_point;
	}
	
	// 3. Find the goal point by moving up the path. If the current point is
	// ever the final point in the path, the search has failed.
	geometry_msgs::Point output; // Just create it once.
	int curr_index = closest_path_index;
	while ( curr_index != last_index ) {
		double x1 = path.points[curr_index].x;
		double y1 = path.points[curr_index].y;
		double x2 = path.points[curr_index+1].x;
		double y2 = path.points[curr_index+1].y;
		
		double a = (x1-x2)*(x1-x2)+(y1-y2)*(y1-y2);
		double b = 2.0*(x1*x2-x1*x1+x1*xr-x2*xr+y1*y2-y1*y1+y1*yr-y2*yr);
		double c = x1*x1+xr*xr+y1*y1+yr*yr-LOOKAHEAD*LOOKAHEAD-2.0*x1*xr-2.0*y1*yr;
		double s = b*b - 4.0*a*c;
		
		// Circle doesn't intersect the line formed by the path.
		if (s < 0.0) {
			curr_index++;
			continue;
		}
		
		// Compute the two possible intersection points.
		double t1 = (sqrt(s)-b)/(2.0*a);
		double t2 = -(sqrt(s)+b)/(2.0*a);
		
		double xt1 = x1 + t1*(x2-x1);
		double yt1 = y1 + t1*(y2-y1);
		double xt2 = x1 + t2*(x2-x1);
		double yt2 = y1 + t2*(y2-y1);
		
		if ( 0.0 < t1 && t1 < 1.0 ) { // t1 is in the segment!
			if ( 0.0 < t2 && t2 < 1.0 ) { // t2 also in the segment!
				double d1 = euclidean(x2, y2, xt1, yt1);
				double d2 = euclidean(x2, y2, xt2, yt2);
				
				if (d1 < d2) { // Return (xt1, yt1)
					output.x = xt1;
					output.y = yt1;
					return output;
				} else { // Return (xt2, yt2)
					output.x = xt2;
					output.y = yt2;
					return output;
				}
			} else { // Return (xt1, yt1)
				output.x = xt1;
				output.y = yt1;
				return output;
			}
		} else if ( 0.0 < t2 && t2 < 1.0 ) { // Return (xt2, yt2)
			output.x = xt2;
			output.y = yt2;
			return output;
		} else { // No valid goal point.
			curr_index++;
			continue;
		}
	}
	
	// Otherwise we've failed to find a valid lookahead point.
	return path.points[closest_path_index]; // Just return closest.
}

// Formats an angle to be between -PI and PI.
// TODO - Move to common
double format_angle( double angle ) {
	while (angle < -M_PI) {
		angle += 2.0*M_PI;
	}
	while (angle > M_PI) {
		angle -= 2.0*M_PI;
	}
	return angle;
}

// Run pure pursuit using the current stored pose and path.
// Publishes motion command using the command_pub member Publisher.
void PurePursuit::purePursuit() {
	geometry_msgs::Point goal = getGoalPoint( _odom.pose.pose, _path );
	
	double xr = _odom.pose.pose.position.x;
	double yr = _odom.pose.pose.position.y;
	double heading = quat_to_yaw( _odom.pose.pose.orientation );
	
	double goal_distance = euclidean( xr, yr, goal.x, goal.y );
	
	geometry_msgs::Twist command;
	command.linear.x = 0.0;
	command.angular.z = 0.0;
	
	// If we're already close, send a 'STOP' command.
	if ( goal_distance < 0.03 ) { // TODO - Move to external parameter
		goal.x = xr;
		goal.y = yr;
		goal_point_pub.publish(goal);
		command_pub.publish(command);
		return;
	}
	
	// First turn to face the goal point if we aren't yet facing it.
	double global_heading = atan2( goal.y - yr, goal.x - xr );
	double relative_heading = format_angle( global_heading - heading );
	
	if ( fabs(relative_heading) > TURNING_ANGLE ) { // Turn in place.
		if ( relative_heading < 0.0 ) { // Turn right.
			command.angular.z = -DEFAULT_W;
		} else { // Turn left.
			command.angular.z = DEFAULT_W;
		}
	} else {
		// Otherwise we drive towards the point.
		double goal_local_x = goal_distance * sin(relative_heading);
		double curvature = 2.0*goal_local_x / (LOOKAHEAD*LOOKAHEAD);
		command.linear.x = DEFAULT_V;
		command.angular.z = curvature*DEFAULT_V;
	}
	
	goal_point_pub.publish(goal);
	command_pub.publish(command);
}
		
