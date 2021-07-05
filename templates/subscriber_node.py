#!/usr/bin/env python

import rospy # Allows use of ROS node functionalities
from std_msgs.msg import String # Allows the use of this message type

def callback( msg ):
    rospy.loginfo( rospy.get_caller_id() + "I heard %s", msg.data )

def listener():
    rospy.init_node( "listener", anonymous = True ) # Anonymous gives duplicate nodes different names so they can both run
    rospy.Subscriber( "chatter", String, callback ) # Subscribe to the chatter topic with the std_msgs.msg.String type. Call callback to handle messages
    rospy.spin() # Keeps Python from exiting the node until it's stopped
    
if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInterruptException: # Catch these exceptions when Ctrl-C is pressed to kill the node
        pass # Do nothing
