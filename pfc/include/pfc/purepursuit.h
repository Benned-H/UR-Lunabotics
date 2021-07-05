// Author: Benned Hedegaard

#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

#include <vector>
#include <cmath>

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "planner/Path.h"

class PurePursuit {

	public:
	
		// TODO - Move these velocity parameters to some MotionModel class
		PurePursuit( const double& lookahead, const double& turn_angle, const double& forward_v, const double& turning_w );
		virtual ~PurePursuit(); // Deconstructor
		
		// Declare message handling functions for the class.
		void handleOdom( const nav_msgs::Odometry::ConstPtr& msg );
		void handlePath( const planner::Path::ConstPtr& msg );
		
		// Declare any ROS publishers. TODO - Move outside the class
		ros::Publisher command_pub;
		ros::Publisher goal_point_pub;
		
	protected:
	
		double quat_to_yaw( const geometry_msgs::Quaternion& q );
		geometry_msgs::Point getGoalPoint( const geometry_msgs::Pose& pose, const planner::Path& path );
		void purePursuit();
		
		bool hasOdom;
		nav_msgs::Odometry _odom;
		bool hasPath;
		planner::Path _path;
		
		double LOOKAHEAD; // Lookahead distance in the pure pursuit algorithm.
		double TURNING_ANGLE; // Turn robot until goals are within this angle.
		double DEFAULT_V; // Default driving speed.
		double DEFAULT_W; // Default turning speed.
};

#endif /* PURE_PURSUIT_H */
