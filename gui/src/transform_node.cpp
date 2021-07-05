// Author: Benned Hedegaard

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

// Broadcasts the robot's pose as a transform.
void odomCallback( const nav_msgs::Odometry& odom ) {
	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;
	
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "world";
	transformStamped.child_frame_id = "robot";
	transformStamped.transform.translation.x = odom.pose.pose.position.x;
	transformStamped.transform.translation.y = odom.pose.pose.position.y;
	transformStamped.transform.translation.z = 0.0;
	transformStamped.transform.rotation = odom.pose.pose.orientation;
	
	// It's like a publisher without a defined topic.
	br.sendTransform(transformStamped);
}

int main( int argc, char* argv[] ) {
	ros::init( argc, argv, "transform_name" );
	ros::NodeHandle node_handle;
	
	// Set up any subscribers
	ros::Subscriber odom_sub = node_handle.subscribe( "simulator/odom", 1, &odomCallback );
	
	sleep(1); // Wait 1 second; gives time for ROS connections to be made.
	
	ros::spin();
	
	return 0;
}
