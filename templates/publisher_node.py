#!/usr/bin/env python

# All Python ROS nodes will have the above line which clarifies that the file should be executed as a Python script

import rospy # Allows use of ROS node functionalities
from std_msgs.msg import String # Allows the use of this message type

def talker():
    pub = rospy.Publisher( "chatter", String, queue_size = 10 ) # Args are topic name, message type, and queue size
    rospy.init_node( "talker", anonymous = True ) # Args are node name and anonymous, which adds random numbers to ensure the name is unique
    rate = rospy.Rate(10) # This object will allow us to sleep at a desired rate, here 10hz

    while not rospy.is_shutdown(): # Checks if the node should stop
        output_msg = String() # Construct the object, then initialize fields
        output_msg.data = "Hello! The time is %s" % rospy.get_time() # We'll send this out
        rospy.loginfo( output_msg ) # Logs the message to the screen, to the log file, and to rosout
        pub.publish( output_msg ) # Publish to the topic
        rate.sleep() # Maintains desired loop rate
        
if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException: # Catch these exceptions when Ctrl-C is pressed to kill the node
        pass # Do nothing
