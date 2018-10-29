#!/usr/bin/env python

import rospy
from baxter_core_msgs.msg import JointCommand

left_command_pub = {}
right_command_pub = {}

def left_pub(commands, names):
    global left_command_pub
    """
    This function sets your speed for the publisher. Use this function to implement your ideas.
    :params:
    x:This is the velocity along x-axis in m/sec. Limits: [-0.3,0.3]
    y:This is the velocity along y-axis in m/sec. Limits: [-0.3,0.3]
    wz: This is the angular velocity for turning (yaw) in rad/sec. Limits: [-1,1]
    :return: Returns a speed message
    """

    joint_cmd = JointCommand()
    joint_cmd.mode = 1
    joint_cmd.command = commands
    joint_cmd.names = names

    left_command_pub.publish(joint_cmd)

def publish_limb():
    global left_command_pub
    """
    :params:
    speed_msg: this is the speed message coming from the set_speed function
    topic: this is the topic name you publish to
    msg_type: this is the message type
    node_name: this is the name of your node
    :return: returns nothing
    """

    #Initialize your node & publisher here.
    rospy.init_node("limb_mover", anonymous=True)
    left_command_pub = rospy.Publisher("/robot/limb/left/joint_command", JointCommand, queue_size=10)
    rate = rospy.Rate(1)

    #Publish till your node is running
    i = 0
    while not rospy.is_shutdown():
        left_pub([i], ['left_w0'])
        i += 0.1
        rate.sleep()


def main():
    """
    This is the main function. Implement your logic here.
    :params: No params
    :return: returns nothing
    """
    publish_limb()

if __name__=="__main__":
    main()
