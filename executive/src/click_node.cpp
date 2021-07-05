// Author: Benned Hedegaard

#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Point.h"

// Simple class to forward clicks from the GUI to executive waypoints.
class ClickHandler{
public:
  ClickHandler() {} // Constructor
  virtual ~ClickHandler() {} // Deconstructor
  void handleClick( const geometry_msgs::PointStamped::ConstPtr& msg );

  ros::Publisher waypoint_pub;
};

void ClickHandler::handleClick( const geometry_msgs::PointStamped::ConstPtr& msg ){
	waypoint_pub.publish( msg->point );
}

int main( int argc, char* argv[] ){	
	ClickHandler click;

	ros::init( argc, argv, "waypoints_node" );
	ros::NodeHandle node_handle;

	ros::Subscriber clicked_point_sub = node_handle.subscribe( "clicked_point", 1, &ClickHandler::handleClick, &click );
	
	// Set up any publishers inside the class instance.
	click.waypoint_pub = node_handle.advertise<geometry_msgs::Point>( "executive/waypoint", 1, true );

	sleep(1); // Wait 1 second; gives time for ROS connections to be made.
	
	ros::spin();
	
	return 0;
}
