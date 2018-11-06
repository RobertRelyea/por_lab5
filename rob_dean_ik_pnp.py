#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Baxter RSDK Inverse Kinematics Example
"""
import argparse
import struct
import sys
import pdb

import rospy

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

import tf
from baxter_core_msgs.msg import JointCommand

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface

right_command_pub = {}


def ik_test(limb, poses):
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()

    ikreq.pose_stamp.append(poses[limb])
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1, 0

    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp_seeds[0], 'None')
        print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
              (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        print "\nIK Joint Solution:\n", limb_joints
        print "------------------"
        print "Response Message:\n", resp
        return 0, resp
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
        return 1, 0

    return 1, 0


def main():
    global right_command_pub
    rospy.init_node("rsdk_ik_service_client")
    right_command_pub = rospy.Publisher("/robot/limb/right/joint_command", JointCommand, queue_size=10)
    
    limb = baxter_interface.Limb("right")
    gripper = baxter_interface.Gripper("right")
    
    #Calibrate your gripper here
    gripper.calibrate()

    point1 = Point( x=0.7,
                    y=-0.15,
                    z=-0.129)

    point2 = Point( x=0.7,
                    y=-0.15,
                    z=0.1)

    point3 = Point( x=0.7,
                    y=-0.5,
                    z=0.1)

    point4 = Point( x=0.7,
                    y=-0.5,
                    z=-0.129)
    gripper.open()
    rospy.sleep(.2)
    
    while(not rospy.is_shutdown()):
        move_joint(limb, point1)
        rospy.sleep(.2)
        gripper.close()
        rospy.sleep(.1)
        move_joint(limb, point2)
        rospy.sleep(.2)
        move_joint(limb, point3)
        rospy.sleep(.2)
        move_joint(limb, point4)
        rospy.sleep(.2)
        gripper.open()
        rospy.sleep(.1)
        move_joint(limb, point3)
        rospy.sleep(.2)
        move_joint(limb, point2)
        rospy.sleep(.2)


def move_joint(limb, point):

    hdr = Header(stamp=rospy.Time.now(), frame_id='base')

    quat = Quaternion( x=-0.0249590815779,
                       y=0.999649402929,
                       z=0.00737916180073,
                       w=0.00486450832011)

    poses = {
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=point,
                orientation=quat
            ),
        ),
    }


    success, resp = ik_test("right", poses)
    if success == 0:
        joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        limb.move_to_joint_positions(joints)
        print("commanded")


if __name__ == '__main__':
    sys.exit(main())
