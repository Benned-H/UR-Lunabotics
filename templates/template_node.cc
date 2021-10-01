/**
 * Implements an example ROS node executable for the UR Lunabotics team.
 * Author: Benned Hedegaard
 */

// Include any packages or definitions needed
// e.g. iostream allows printing, ROS allows defining the node.
#include <iostream>

#include "ros/ros.h"

// Include any message types you'll be publishing, e.g.
#include "nav_msgs/OccupancyGrid.h"
#include "package_name/DataType.h"

// Make sure to include the package's header file if you use its class.
#include "package_name/package.h"

int main( int argc, char* argv[] ){
	ClassName obj( a, b, c ); // Declare an instance of the relevant class, if needed.
	// Give the necessary inputs if the class has any.
	
	ros::init( argc, argv, "node_name" ); // e.g. "planner_node"
	ros::NodeHandle node_handle; // We use this to set up ROS connections.
	
	// Set up any subscribers
	ros::Subscriber datatype_sub = node_handle.subscribe( "topic_name", 1, &ClassName::handleMessageType, &obj );
	/* The topic name needs to match exactly; it might be package/topic_name
	depending on the corresponding publisher. Then we set the queue
	size, or number of messages to back up if we're processing too slowly,
	and define a method and class instance to handle incoming messages. */
	
	// Set up any publishers inside the class instance.
	obj.datatype_pub = node_handle.advertise<package_name::DataType>( "topic_name", 1, true );
	/* We choose the topic name here; agree upon it across packages. The number
	is the publishing queue size, or how many messages to hold on to. Then we
	set latching to true, which means that the topic will save its latest
	message and send it to new subscribers when they connect. */
	
	// Set up node-specific publishers.
	ros::Publisher datatype_pub = node_handle.advertise<package_name::DataType>( "topic_name2", 1, true ); // Same meaning as above.
		
	sleep(1); // Wait 1 second; gives time for ROS connections to be made.
		
	// You could then instantiate, fill in, and publish a message without needing the package's class at all:
	package_name::DataType msg;
	msg.x = 1.0;
	msg.y = 2.0;
	datatype_pub.publish(msg); // Publishes the message msg.
	
	// We then can make the node do a few different things:
	
	/* Option 1: Spin - The node will just sit without closing and handle
	incoming messages, publishing only when its internal methods publish to
	any of its member variable Publishers. Waits until manually closed. */
	ros::spin();
	
	/* Option 2: Exit - Wait a second (allows easier debugging) then exit. */
	sleep(1);
	
	/* Option 3: Update and publish - We use a ROS timer to periodically
	update the class' member variables and publish them after each step. */
	double frequency = 25.0; // Desired rate of the timer in Hz
	ros::Rate timer(frequency);
	
	// Keeps looping as long as the node is running.
  while( ros::ok() ){
    ros::spinOnce(); // Calls all waiting callbacks, i.e. handles messages.
    obj.step( 1.0 / frequency ); // Advance the class' internal simulation/belief.
    datatype_pub.publish(data); // data could be a class member variable.
    timer.sleep(); // Waits rest of cycle to assure proper Hz
  }
	
  // In all cases, the following exits the execution of the node:
  return 0;
}
