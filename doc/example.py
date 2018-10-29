#!/usr/bin/env python

"""
Script to return Baxter's arms to a "home" position
"""


import rospy


import baxter_interface


rospy.init_node('Home_Arms')


limb_right = baxter_interface.Limb('right')

gripper = baxter_interface.Gripper('right')

home_right = {'right_s0': 0.09, 'right_s1': -1.00, 'right_w0': -0.67, 'right_w1': 1.03, 'right_w2': 0.50, 'right_e0': 1.18, 'right_e1': 1.94}
arm_up = {'right_s1': 0}
arm_down = {'right_s1': 1}
arm_right = {'right_s0': -1}
arm_left = {'right_s0': 0}
fix_elbow = {'right_e0': 0, 'right_e1' : 0, 'right_w0': 0}


gripper.calibrate()
gripper.open()

print("homing")
limb_right.move_to_joint_positions(home_right)
gripper.open()

print("fix elbow")
limb_right.move_to_joint_positions(fix_elbow)
gripper.open()

ik_left_down = {'right_s0': -0.41477990196901043, 'right_s1': -0.6129804557907225, 'right_w0': -1.2322210318017135, 'right_w1': 0.4579743260014188, 'right_w2': 0.8953404137971069, 'right_e0': 0.6527511985851052, 'right_e1': 1.4934465270857933}

ik_left_up
ik_right_up
ik_right_down




while not rospy.is_shutdown():


    rospy.sleep(0.2)
    print("move down")
    limb_right.move_to_joint_positions(arm_down)
    gripper.close()

    rospy.sleep(0.2)
    print("move up")
    limb_right.move_to_joint_positions(arm_up)

    rospy.sleep(0.2)
    print("move left")
    limb_right.move_to_joint_positions(arm_left)

    rospy.sleep(0.2)
    print("move down")
    limb_right.move_to_joint_positions(arm_down)
    gripper.open()

    rospy.sleep(0.2)
    print("move up")
    limb_right.move_to_joint_positions(arm_up)

    rospy.sleep(0.2)
    print("move right")
    limb_right.move_to_joint_positions(arm_right)





    print("moved")	

    
    
    

    


