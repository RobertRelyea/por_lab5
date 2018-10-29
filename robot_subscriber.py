#!/usr/bin/env python
#This line is very important


"""
Intro goes here
"""

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from baxter_core_msgs.msg import AssemblyState

def joint_states_sub(data):
    """
    Callback does something everytime a message is recieved by a subscriber.
    You define the logic after this line.
    :params:
    data: this is the data processed by the callback
    :return: returns nothing
    """
    print(data.position)
    # print("Got joint states")

def state_sub(data):
    """
    Callback does something everytime a message is recieved by a subscriber.
    You define the logic after this line.
    :params:
    data: this is the data processed by the callback
    :return: returns nothing
    """
    if data.enabled:
    	print("Robot enabled")
    else:
    	print("Robot disabled")
    # print("Got robot state")

def subscribe():
    rospy.init_node("robot_subscriber", anonymous=True)
    rospy.Subscriber('/robot/joint_states', JointState, joint_states_sub)
    rospy.Subscriber('/robot/state', AssemblyState, state_sub)
    rospy.spin() # keeps the subscriber process from dying.

def main():
    """
    You can begin by modifying this function.
    """
    subscribe()

if __name__=="__main__":
    main()
