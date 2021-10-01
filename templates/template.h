/**
 * Example header file template for the UR Lunabotics team
 * Author: Benned Hedegaard
 */

#pragma once

// Include all (standard) library dependencies, e.g.
#include <iostream>
#include <vector>
#include <Eigen/Dense> // etc.

// Include all required header files.
// Including ROS lets us define publishers as member variables.
#include "ros/ros.h"
#include "package_name/DataType.h" // etc.

class ClassName{
public: // These data members can be accessed by other classes.

  /**
      ClassName class constructor from [explain what is passed in here].
      
      @brief ClassName class constructor from [explain what is passed in here].
      @param[in]    worldModelArg     [explanation of the argument here]
      @returns      none
      @throws       no expected throws
  */
  ClassName( const std::shared_ptr<WorldModel>& worldModelArg );

  /**
      Default ClassName destructor.
  */
  virtual ~ClassName() = default;

  /**
      Handles a DataType message by [explain].

      @brief Handles a DataType message by [explain].
      @param[in]    msg       DataType message to be processed
      @returns      none
      @throws       std::runtime_error on invalid message data
  */
  void handleMessageType( const package_name::DataType::ConstPtr& msg );
		
  /* ROS Publisher for DataType objects */
  ros::Publisher datatype_pub;

  /* Explain each member variable in a comment like this. */
  double member_variable;
	
protected: // These data members are inaccessible outside the class.

  /**
      Draws a circle to the GUI with the given parameters.

      @brief Draws a circle to the GUI with the given parameters.
      @param[in]    x     x coord of the drawn circle
      @param[in]    y     y coord of the drawn circle
      @returns      none
      @throws       no ewxpected throws
  */
  void drawCircle( const double& x, const double& y, const double& radius ); // e.g. for a GUI
		
  /* You can also include private member variables. Document these too! */
  package_name::DataType data;

  /* Indicates if the condition is true */
  bool condition;
};
