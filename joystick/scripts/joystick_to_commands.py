#!/usr/bin/env python

# All Python ROS nodes will have the above line which clarifies that the file should be executed as a Python script

import rospy # Allows use of ROS node functionalities
from inputs import get_gamepad
from geometry_msgs.msg import Twist # Allows the use of this message type

def joystick_to_commands():
    pub = rospy.Publisher( "motion_commands", Twist, queue_size = 1 ) # Args are topic name, message type, and queue size
    rospy.init_node( "joystick_to_commands", anonymous = False ) # No need to make node anonymous
    
    current_x = 0 # Store most recent x and y joystick outputs
    current_y = 0
    send_new_command = False # Only send new commands when we have joystick changes
    x_scale = 3.14 / -32768.0 # Scale between 0 and pi
    y_scale = 1.5 / -32768.0 # Scale between 0 and 1.5
    
    while not rospy.is_shutdown(): # Checks if the node should stop
    
        events = get_gamepad()
        for event in events:
            if event.code == "ABS_RX":
                if abs( event.state ) > 1500: # Less than 1500 should be sent to 0
                    current_x = event.state # Type is int
                else: # Round x to 0
                    current_x = 0
                send_new_command = True
            if event.code == "ABS_RY":
                if abs( event.state ) > 1500: # Less than 1500 should be sent to 0
                    current_y = event.state # Type is int
                else: # Round y to 0
                    current_y = 0
                send_new_command = True
        if send_new_command:
            rospy.loginfo( str(current_x) + " " + str(current_y) )
            
            command_msg = Twist()
            command_msg.linear.x = y_scale * current_y
            command_msg.angular.z = x_scale * current_x
            pub.publish( command_msg ) # Publish to the topic
            
            send_new_command = False # Now wait for next joystick change
            
if __name__ == "__main__":
    try:
        joystick_to_commands()
    except rospy.ROSInterruptException: # Catch these exceptions when Ctrl-C is pressed to kill the node
        pass # Do nothing
