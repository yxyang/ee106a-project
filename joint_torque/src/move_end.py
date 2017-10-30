#!/usr/bin/env python

# Copyright (c) 2013-2017, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
SDK Joint Position Example: keyboard
"""
import argparse

import rospy

import intera_interface
import intera_external_devices
import time

from intera_interface import CHECK_VERSION
from geometry_msgs.msg import Wrench, Vector3

default_positions = eval(open('parsed.txt', 'r').read())
default_positions = [default_positions[1400], default_positions[1600]]

def move_joints(side):
    limb = intera_interface.Limb(side)

    joints = limb.joint_names()

    # Publish end-effector torques to a topic
    pub_torque = rospy.Publisher('endpoint_torque', Wrench, queue_size = 10)

    action_count = 0; i = 0;
    while not rospy.is_shutdown():
        force = limb.endpoint_effort()['force']
        torque = limb.endpoint_effort()['torque']
        w = Wrench(Vector3(force.x, force.y, force.z), Vector3(torque.x, torque.y, torque.z));
        pub_torque.publish(w);

        action_count += 1;
        if action_count % 10000 != 0:
            continue
        else:
            action_count = 0
        # Move joints according to specified locations
        print(force)
        i += 1;
        if (i >= len(default_positions)):            
            i = 0;        
        raw_input("Press Enter to continue..." + str(i))
        limb.set_joint_positions(default_positions[i]);
        diff = sum([(limb.joint_angle(key) - default_positions[i][key])** 2 for key in default_positions[i]])
        while (diff > 0.0001):
            limb.set_joint_positions(default_positions[i])
            diff = sum([(limb.joint_angle(key) - default_positions[i][key])** 2 for key in default_positions[i]])
            time.sleep(0.005)



def main():
    epilog = """
See help inside the example with the '?' key for key bindings.
    """
    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    if not valid_limbs:
        rp.log_message(("Cannot detect any limb parameters on this robot. "
                        "Exiting."), "ERROR")
        return

    print("Initializing node... ")
    rospy.init_node("sdk_joint_position_keyboard")
    print("Getting robot state... ")
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example.")

    rospy.on_shutdown(clean_shutdown)

    rospy.loginfo("Enabling robot...")
    rs.enable()
    move_joints(valid_limbs[0])
    print("Done.")


if __name__ == '__main__':
    main()
