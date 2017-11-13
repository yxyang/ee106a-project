#!/usr/bin/env python
from ar_track_alvar_msgs.msg import AlvarMarkers
import intera_external_devices
import intera_interface
from intera_interface import CHECK_VERSION
import numpy as np
import rospy
import time
import tf


class doctor_sawyer:
    def __init__(self):
        self.ar_track_data = []
        self.table_x, self.table_y, self.table_z = 0, 0, 0

    # call back function for ar tag tracking
    def ar_track_callback(self, data):
        # If already get enough data, ignore it
        if len(self.ar_track_data) > 10:
            return

        pos = data.markers[0].pose.pose.position
        self.ar_track_data.append((pos.x, pos.y, pos.z))

        if len(self.ar_track_data) == 10:
            # Average the received data
            # might need to filter out outliers in the future
            self.table_x = np.mean([pos[0] for pos in self.ar_track_data])        
            self.table_y = np.mean([pos[1] for pos in self.ar_track_data])        
            self.table_z = np.mean([pos[2] for pos in self.ar_track_data])        
            print("Table center found at: " + str([self.table_x, self.table_y, self.table_z]))
#            listener = tf.TransformListener()
#            print(listener.allFramesAsString())
#
#            while not rospy.is_shutdown():
#                try:
#                    (trans,rot) = listener.lookupTransform('/head_camera', '/base', rospy.Time(0))
#                    break
#                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#                    continue
    
            # Look up TF transform
#            g = listener.fromTranslationRotation(trans, rot)
#            point_camera_frame = np.array([camera_x, camera_y, camera_z, 1])
#            print(point_camera_frame)
#            print(g.dot(point_camera_frame))


    # Find an AR tag that designates the center of camera
    def find_table(self):
        # Capture output from topic 'visualization_marker'
        rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.ar_track_callback)

        # Call TF to transform to coordinate w.r.t. base frame

    # Outputs a 2D array of hardness in each point probed
    def probe(self):
        pass

    # Outputs temperature measure
    def measure_temperature(self):
        pass

    def measure_pulse(self):
        pass


def main():
    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    rospy.init_node('doctor', anonymous=True)        

    rs = intera_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    rs.enable()

    doctor = doctor_sawyer()
    doctor.find_table()
    rospy.spin()



if __name__ == '__main__':
    main()
