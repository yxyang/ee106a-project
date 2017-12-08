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
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
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

    def move_gripper_to(self, x, y, z, orientation = [0,-1,0,0], timeout = 15, confirmation = True):
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
                    x=orientation[0],
                    y=orientation[1],
                    z=orientation[2],
                    w=orientation[3],
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
        dz_start = 0.4
        rospy.sleep(1)
        self.move_gripper_to(x, y, self.table_z + dz_start, confirmation = False)
        #Start poking down in the z-direction using torque control
        rospy.sleep(0.5)
        force = self.limb.endpoint_effort()['force']

        #Keep poking down as long as force within threshold
        thres = 6.0 #threshold in Newton
        z = self.table_z + dz_start;
        while (force.z < thres):
            rospy.sleep(0.2)
            force = self.limb.endpoint_effort()['force']
            rospy.logdebug("End Effector Force is: " + str([force.x, force.y, force.z]))            
    
            #Execute the plan
            z -= 0.01
            self.move_gripper_to(x, y, z, confirmation = False, timeout = 0.5)

        self.move_gripper_to(x, y, self.table_z + dz_start, confirmation = False)
        return (z, force.z)

    def poke_at_with_avg(self, x, y):    
        dz_start = 0.3
        rospy.sleep(0.5)
        self.move_gripper_to(x, y, self.table_z + dz_start, confirmation = False)
        #Start poking down in the z-direction using torque control
        rospy.sleep(0.5)
        force = self.limb.endpoint_effort()['force']

        #Keep poking down as long as force within threshold
        thres = 6.0 #threshold in Newton
        z = self.table_z + dz_start;
        force_z = 0
        while (force_z < thres):
            rospy.sleep(0.4)

            force_measurements = []
            # Measure the force 100 times
            for _ in range(100):
                force = self.limb.endpoint_effort()['force']
                force_measurements.append(force.z)

            force_z = np.mean(force_measurements)
            rospy.logdebug("End Effector Force_z: " + str(force_z))            
    
            #Execute the plan
            z -= 0.01
            self.move_gripper_to(x, y, z, confirmation = False, timeout = 0.5)

        self.move_gripper_to(x, y, self.table_z + dz_start, confirmation = False)
        return (z, force.z)

    def poke_at_with_min_max(self, x, y):
        dz_start = 0.3
        self.move_gripper_to(x, y, self.table_z + dz_start, confirmation = False)
        #Start poking down in the z-direction using torque control
        raw_input("Press any key to continue")

        #STEP 1: keep poking down until minimum threshold is reached
        thres_min = 1 #threshold in Newton
        z = self.table_z + dz_start;
        force_z = -0.1
        while (force_z < thres_min):
            rospy.sleep(0.2)

            force_measurements = []
            # Measure the force 10 times
            for _ in range(100):
                force = self.limb.endpoint_effort()['force']
                force_measurements.append(force.z)

            force_z = np.mean(force_measurements)
            rospy.logdebug("End Effector Force_z: " + str(force_z))            
    
            #Execute the plan
            z -= 0.01
            self.move_gripper_to(x, y, z, confirmation = False, timeout = 0.5)

        rospy.logwarn("Reached surface at z=" + str(z)) 
        raw_input("Press any key to continue")

        #STEP 2: keep poking down until maximum threshold is reached
        thres_max = 6.0
        z_start = z
        force_z = 0
        while (force_z < thres_max):
            rospy.sleep(0.2)

            force_measurements = []
            # Measure the force 10 times
            for _ in range(10):
                force = self.limb.endpoint_effort()['force']
                force_measurements.append(force.z)

            force_z = np.mean(force_measurements)
            rospy.logdebug("End Effector Force_z: " + str(force_z))            
    
            #Execute the plan
            z -= 0.01
            self.move_gripper_to(x, y, z, confirmation = False, timeout = 0.5)

        z_end = z
        rospy.logwarn("Reached maximum force at z=" + str(z_end) + "dz=" + str(z_end - z_start)); 
        raw_input("Press any key to continue")

        self.move_gripper_to(x, y, self.table_z + dz_start, confirmation = False)
        return (z_end - z_start, z_start) #(Hardness, Topology)


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

    def move_to_neutral(self):
        rospy.logdebug("Moving to neutral position")
        self.limb.move_to_joint_positions({
            'right_j6': 0, 
            'right_j5': 0, 
            'right_j4': 0, 
            'right_j3': 0, 
            'right_j2': 0, 
            'right_j1': 0, 
            'right_j0': 0})
        self.head.set_pan(0.0)
        rospy.sleep(2)

    # Outputs a 2D array of hardness in each point probed
    # Parameters: nx, ny, dx, dy
    # Probes at [table_x-nx*dx, table_x + nx*dx]
    # Returns a dictionary with key as (x, y) and value as something representing hardness
    def probe(self, nx, ny, dx, dy):
        #Initialize arms
        self.move_to_neutral();

        data = {} # start with empty dictionary
        for i in range(-nx, nx+1):
            for j in range(-nx, nx+1):
                data[(i, j)] = self.poke_at_with_avg(self.table_x + i * dx, self.table_y + j * dy)
                rospy.logdebug("Starting to probe in a new position") 

        self.move_to_neutral()
        self.move_to_standby()

        print("Plotting results...")
        x = np.arange(-nx, nx+1, 1)
        y = np.arange(-ny, ny+1, 1)
        x2d, y2d = np.meshgrid(x, y)
        x, y = x2d.ravel(), y2d.ravel()
        z = np.zeros_like(x) * 0.0
        for i in range(x.shape[0]):
            #print(data[(x[i], y[i])][0])
            z[i] = data[(x[i], y[i])][0]
        #print(z)
        fig = plt.figure(figsize=(4, 6))
        ax1 = fig.add_subplot(1, 1, 1, projection='3d')
        ax1.bar3d(x, y, 0, 1, 1, z - np.min(z), shade=True)
        ax1.set_title('Tumor Detection Results --- distance probed')
        plt.show()

        x = np.arange(-nx, nx+1, 1)
        y = np.arange(-ny, ny+1, 1)
        x2d, y2d = np.meshgrid(x, y)
        x, y = x2d.ravel(), y2d.ravel()
        z = np.zeros_like(x) * 0.0
        for i in range(x.shape[0]):
            #print(data[(x[i], y[i])][0])
            z[i] = data[(x[i], y[i])][1]
        #print(z)
        fig = plt.figure(figsize=(4, 6))
        ax1 = fig.add_subplot(1, 1, 1, projection='3d')
        ax1.bar3d(x, y, 0, 1, 1, z - np.min(z), shade=True)
        ax1.set_title('Tumor Detection Results --- force')
        plt.show()
        return data

    # 1) Move to a position relative to the AR tag and open the gripper
    # 2) Put the box under gripper
    # 3) Press a button to close the gripper and start measuring
    # 4) At the press of a button, open the gripper
    # 5) At the press of a button, close the gripiper again and return (to get ready for the next task)
    def start_measure(self):
        self.move_to_neutral()
        self.move_gripper_to(self.table_x, self.table_y, 0.1, confirmation = False,
                             orientation = [-0.42825, 0.60241, -0.43926, 0.51064])
        right_gripper = intera_interface.gripper.Gripper('right')
        right_gripper.calibrate()
        right_gripper.open()
        raw_input("Now put the box inside the gripper, and press any key to close it")
        right_gripper.close()
        raw_input("When you're done, press any key to open the gripper and release the box")
        right_gripper.open()
        raw_input("Press any key to close and move to standby position")
        right_gripper.close()
        self.move_to_standby()

def main():
    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    rospy.init_node('doctor', log_level=rospy.DEBUG)        

    rs = intera_interface.RobotEnable(CHECK_VERSION)

    head_display = intera_interface.HeadDisplay()
    head_display.display_image("./head_display_pics/redcross.png", False, 1.0)
    init_state = rs.state().enabled

    rs.enable()

    rospy.logdebug("Closing gripper...")
    #raw_input("Press any key to close gripper")
    right_gripper = intera_interface.gripper.Gripper('right')
    right_gripper.calibrate()    
    #rospy.sleep(2.0)
    right_gripper.open()
    #raw_input("Press any key to close gripper")
    right_gripper.close()
    #rospy.sleep(2.0)
    rospy.logdebug("Gripper closed")
    doctor = doctor_sawyer(valid_limbs[0])
    doctor.find_table()


    while (not rospy.is_shutdown()):
        action = raw_input("(P)oke, (M)easurement?")
        if (action == "P"):
            print(doctor.probe(2, 2, 0.02, 0.02))
        if (action == "M"):
            doctor.start_measure()
        if (action == "P1"):
            doctor.probe(0, 0, 0.02, 0.02)

    rospy.spin()


if __name__ == '__main__':
    main()
