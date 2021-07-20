// Author: Benned Hedegaard

#pragma once

#include <Eigen/Dense>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "simulator/Obstacles.h"
#include "sensor_msgs/LaserScan.h"

class Simulator{
public:
	
  Simulator(); // Constructor
  virtual ~Simulator() = default; // Default deconstructor
		
  // Declare message handling functions for the class.
  /**
    TODO - Update this bro
  */
  void handleMotionCommand( const geometry_msgs::Twist::ConstPtr& msg );
  void handleObstacles( const simulator::Obstacles::ConstPtr& msg );
		
  void step( const double& dt );
  nav_msgs::Odometry getOdometry();
  sensor_msgs::LaserScan getScan( int beams );
		
protected:
// TODO - Create a custom robot state and motion command type.

  Eigen::Vector2d u; // Current motion command.
  Eigen::Vector3d x; // Current robot pose.
  simulator::Obstacles obstacles; // Simulated obstacles.
};
