// Author: Benned Hedegaard

#pragma once

// Include all (standard) library dependencies, e.g.
#include <iostream>
#include <vector>
#include <Eigen/Dense> // etc.

// Include all necessary header files.
// Including ROS lets us define publishers as member variables.
#include "ros/ros.h"
#include "package_name/DataType.h" // etc.

class ClassName{
public: // These data members can be accessed by other classes.
	
  ClassName(); // Constructor; could have input variables.
  virtual ~ClassName() = default; // Default deconstructor

  // Declare message handling functions for the class.
  void handleMessageType( const package_name::DataType::ConstPtr& msg );
		
  // Declare any ROS publishers.
  ros::Publisher datatype_pub;
		
  double member_variable; // e.g. some hyperparameter for the class.
	
protected: // These data members are inaccessible outside the class.

  // Include any internal functions the class needs.
  void drawCircle( double radius ); // e.g. for a GUI
		
  // Also include private member variables.
  package_name::DataType data;
  bool condition;
};
