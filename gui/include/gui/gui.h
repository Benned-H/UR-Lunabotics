// Author: Benned Hedegaard

#ifndef GUI_H
#define GUI_H

#include "ros/ros.h"
#include "std_msgs/ColorRGBA.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "planner/Path.h"
#include "simulator/Obstacles.h"
#include "visualization_msgs/Marker.h"

class GUI {

	public:
	
		// TODO - Where is a model of the robot stored? Standardize
		GUI( const double& diameter ); // Robot diameter (m) is included.
		virtual ~GUI(); // Deconstructor
		
		void handleOdom( const nav_msgs::Odometry::ConstPtr& msg );
		void handlePath( const planner::Path::ConstPtr& msg );
		void handleLookaheadPoint( const geometry_msgs::Point::ConstPtr& msg );
		void handleObstacles( const simulator::Obstacles::ConstPtr& msg );
		
		std_msgs::ColorRGBA color( const double& r, const double& g, const double& b, const double& a );
		void update();
		
		ros::Publisher marker_pub;
		
	protected:
	
		void drawPose( const geometry_msgs::Pose& pose, double r, double g, double b );
		void drawPath( const planner::Path& path, double r, double g, double b );
		void drawLookahead( const geometry_msgs::Point& point, double radius, double r, double g, double b );
		void drawObstacles( double r, double g, double b );
		
		geometry_msgs::Pose _pose;
		bool hasPath;
		planner::Path _path;
		bool hasLookahead;
		geometry_msgs::Point _lookahead;
		simulator::Obstacles _obstacles;
		
		double ROBOT_DIAMETER;
};

#endif /* GUI_H */
