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

import rospy

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from baxter_interface import CHECK_VERSION
import baxter_interface
import baxter_external_devices

def ik_test(limb):
    rospy.init_node("rsdk_ik_service_client")
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        'left': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=0.657579481614,
                    y=0.851981417433,
                    z=0.0388352386502,
                ),
                orientation=Quaternion(
                    x=-0.366894936773,
                    y=0.885980397775,
                    z=0.108155782462,
                    w=0.262162481772,
                ),
            ),
        ),
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=0.656982770038,
                    y=-0.852598021641,
                    z=0.0388609422173,
                ),
                orientation=Quaternion(
                    x=0.367048116303,
                    y=0.885911751787,
                    z=-0.108908281936,
                    w=0.261868353356,
                ),
            ),
        ),
    }

    ikreq.pose_stamp.append(poses[limb])
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

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

        # Move joints to the position calculated by IK server
        rs = baxter_interface.RobotEnable(CHECK_VERSION)
        rs.enable()
        limb = baxter_interface.Limb(limb)
        limb.set_joint_positions(limb_joints);
        diff = sum([(limb.joint_angle(key) - limb_joints[key])** 2 for key in limb_joints])
        while (diff > 0.0001):
            limb.set_joint_positions(limb_joints)
            diff = sum([(limb.joint_angle(key) - limb_joints[key])** 2 for key in limb_joints])
            #time.sleep(0.005)
        raw_input("Press any key to continue")
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")

    return 0

def ik_test_probe():
    limb = 'right'
    rospy.init_node("ik_test_probe")
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"

    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    rs.enable()
    limb = baxter_interface.Limb(limb)

    x0, y0, z0 = 0.7, -0.6, 0.1;
    dx, dy, dz = 0.06, 0.06, -0.02;

    for ix in range(-2, 3):
        for iy in range(-2, 3):
            for iz in range(0, 5):
                hdr = Header(stamp=rospy.Time.now(), frame_id='base')            
                pose = PoseStamped(
                        header=hdr,
                        pose=Pose(
                            position=Point(
                                x=x0 + ix * dx,
                                y=y0 + iy * dy,
                                z=z0 + iz * dz,
                            ),
                            orientation=Quaternion(
                                x=0.0,
                                y=-1.0,
                                z=0.0,
                                w=0.0,
                            ),
                        ),
                    )
                
                iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
                ikreq = SolvePositionIKRequest()
                ikreq.pose_stamp.append(pose)
                try:
                    rospy.wait_for_service(ns, 5.0)
                    resp = iksvc(ikreq)
                except (rospy.ServiceException, rospy.ROSException), e:
                    rospy.logerr("Service call failed: %s" % (e,))
                    return 1
            
                # Check if result valid, and type of seed ultimately used to get solution
                # convert rospy's string representation of uint8[]'s to int's
                print("ix, iy, iz = " + str([ix, iy, iz]))
                resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                                           resp.result_type)
                if (resp_seeds[0] != resp.RESULT_INVALID):       
                    limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))       
                    # Move joints to the position calculated by IK server
                    print(sum([(limb.joint_angle(key) - limb_joints[key])** 2 for key in limb_joints]))

                    limb.move_to_joint_positions(limb_joints, timeout=2);
                    #raw_input("Press any key to move")
                    print(limb.endpoint_effort()['force'])
                    diff = sum([(limb.joint_angle(key) - limb_joints[key])** 2 for key in limb_joints])
                    
                else:
                    print("INVALID POSE - No Valid Joint Solution Found.")

    return 0

def main():
    return ik_test_probe()

if __name__ == '__main__':
    sys.exit(main())
