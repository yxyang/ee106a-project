#!/usr/bin/env python
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion
)
import intera_external_devices
import intera_interface
from intera_interface import CHECK_VERSION
from intera_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
import numpy as np
import rospy
from std_msgs.msg import Header
import sys
import time
import tf

class doctor_sawyer:
    def __init__(self, side):
        self.ar_track_data = []
        self.table_x, self.table_y, self.table_z = 0, 0, 0
        self.limb = intera_interface.Limb(side)
        self.head = intera_interface.Head();
        self.move_to_standby();

    # call back function for ar tag tracking
    def ar_track_callback(self, data):
        # If already get enough data, ignore it
        if len(self.ar_track_data) > 10:
            return

        if len(data.markers) == 0:
            return

        rospy.logdebug("Received AR track data")
        pos = data.markers[0].pose.pose.position
        self.ar_track_data.append((pos.x, pos.y, pos.z))

        if len(self.ar_track_data) == 10:
            # Average the received data
            # might need to filter out outliers in the future
            self.table_x = np.mean([pos[0] for pos in self.ar_track_data])        
            self.table_y = np.mean([pos[1] for pos in self.ar_track_data])        
            self.table_z = np.mean([pos[2] for pos in self.ar_track_data])    
            rospy.logwarn("Table center found at: %s", str([self.table_x, self.table_y, self.table_z]))   


    # Find an AR tag that designates the center of camera
    def find_table(self):
        # Capture output from topic 'visualization_marker'
        rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.ar_track_callback)

        # Call TF to transform to coordinate w.r.t. base frame

    def move_gripper_to(self, x, y, z, timeout = 15, confirmation = True):
        ns = "ExternalTools/right/PositionKinematicsNode/IKService"
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        pose = PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=x,
                    y=y,
                    z=z,
                ),
                orientation=Quaternion(
                    x=0,
                    y=-1,
                    z=0,
                    w=0,
                ),
            ),
        )
        ikreq.pose_stamp.append(pose)
        ikreq.tip_names.append('right_hand')

        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            print(repr(e))
            #rospy.logerr("Service call failed: %s" % (e,))
            return 1

        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))       
        # Move joints to the position calculated by IK server
        

        #Execute the plan
        if confirmation:
            print("Difference between current joint angle and target joint angle is:")
            print([(self.limb.joint_angle(key) - limb_joints[key])for key in limb_joints])
            confirmed = raw_input("Execute? (y/n)")
        else:
            confirmed = "y"


        if confirmed == "y":
            self.limb.move_to_joint_positions(limb_joints, timeout = timeout);
        else:
            print("Aborted")

    def poke_at(self, x, y):       
        self.move_gripper_to(x, y, self.table_z + 0.3, confirmation = False)
        #Start poking down in the z-direction using torque control
        rospy.sleep(1)
        force = self.limb.endpoint_effort()['force']
        torque = self.limb.endpoint_effort()['torque']

        #Keep poking down as long as force within threshold
        thres = 5.0 #threshold in Newton
        z = self.table_z + 0.3;
        while (force.z < thres):
            rospy.sleep(1)
            force = self.limb.endpoint_effort()['force']
            torque = self.limb.endpoint_effort()['torque']
            rospy.logdebug("End Effector Force is: " + str([force.x, force.y, force.z]))            
    
            #Execute the plan
            z -= 0.02
            self.move_gripper_to(x, y, z, confirmation = False, timeout = 1)


    def move_to_standby(self):
        # Return to standby position at the end of operation
        rospy.logdebug("Moving to standby position...")
        self.limb.move_to_joint_positions({
            'right_j6': 0.9906259765625, 
            'right_j5': 2.26457421875, 
            'right_j4': 0.4366396484375, 
            'right_j3': -2.0645771484375, 
            'right_j2': -0.205685546875, 
            'right_j1': 0.8053369140625, 
            'right_j0': -2}
            )
        self.head.set_pan(-2 * np.pi + 2)
        rospy.sleep(3)

    # Outputs a 2D array of hardness in each point probed
    # Parameters: nx, ny, dx, dy
    # Probes at [table_x-nx*dx, table_x + nx*dx]
    # Returns a dictionary with key as (x, y) and value as something representing hardness
    def probe(self, nx, ny, dx, dy):
        #Initialize arms
        self.limb.move_to_joint_positions({
            'right_j6': 0, 
            'right_j5': 0, 
            'right_j4': 0, 
            'right_j3': 0, 
            'right_j2': 0, 
            'right_j1': 0, 
            'right_j0': 0})
        self.head.set_pan(0.0)

        ans = {} # start with empty dictionary
        for i in range(-nx, nx+1):
            for j in range(-nx, nx+1):
                ans[(i, j)] = self.poke_at(self.table_x + i * dx, self.table_y + j * dy)
                rospy.logdebug("Starting to probe in a new position") 

        self.move_to_standby();
        return ans

    # Outputs temperature measure
    def measure_temperature(self):
        pass

    def measure_pulse(self):
        pass


def main():
    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    rospy.init_node('doctor', log_level=rospy.DEBUG)        

    rs = intera_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    rs.enable()

    rospy.logdebug("Closing gripper...")
    #raw_input("Press any key to close gripper")
    right_gripper = intera_interface.gripper.Gripper('right')
    right_gripper.calibrate()    
    #rospy.sleep(2.0)
    #raw_input("Press any key to close gripper")
    right_gripper.close()
    #rospy.sleep(2.0)
    rospy.logdebug("Gripper closed")
    doctor = doctor_sawyer(valid_limbs[0])
    doctor.find_table()


    while (not rospy.is_shutdown()):
        action = raw_input("(P)oke, (H)eartbeat, (T)emperature?")
        if (action == "P"):
            print(doctor.probe(2, 2, 0.05, 0.05))

    rospy.spin()


if __name__ == '__main__':
    main()
