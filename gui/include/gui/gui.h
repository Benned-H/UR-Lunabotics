// Author: Benned Hedegaard

#pragma once

#include "ros/ros.h"
#include "std_msgs/ColorRGBA.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "planner/Path.h"
#include "simulator/Obstacles.h"
#include "visualization_msgs/Marker.h"

class GUI{
public:

  // TODO - Where is a model of the robot stored? Standardize
  GUI( const double& diameter ); // Robot diameter (m) is included.
  
  virtual ~GUI() = default; // Default deconstructor

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

  // TODO - Use std::optional rather than extra booleans
  geometry_msgs::Pose pose;
  bool hasPath;
	planner::Path path;
	bool hasLookahead;
	geometry_msgs::Point lookahead;
	simulator::Obstacles obstacles;
		
  double ROBOT_DIAMETER;
};
