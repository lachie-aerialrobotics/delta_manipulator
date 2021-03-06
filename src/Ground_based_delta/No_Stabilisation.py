#! /usr/bin/env python
import rospy
import numpy as np

import tf2_ros
from quaternion_functions import qv_mult, q_mult, q_conjugate
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped

#script calculates required end-effector position of manipulator to stabilise base perturbations using tf2 frames
class Controller:
    def __init__(self):
        #broadcast static frames
        br_static = tf2_ros.StaticTransformBroadcaster()
        tf_base2manip = transform_msg(
            "base_link", "manipulator",
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0
        )
        self.q = quaternion_from_euler(np.pi, 0.0, np.pi/2)
        tf_map2base = transform_msg(
            "map", "base_link",
            0.0, 0.0, 0.0,
            self.q[0], self.q[1], self.q[2], self.q[3]
        )
  
        br_static.sendTransform([tf_map2base, tf_base2manip])

        #publish platform setpoint
        self.pub_tip_pos = rospy.Publisher('/tooltip/setpoint_position/local', PointStamped, queue_size=1, tcp_nodelay=True) 

        #subscribe to tooltip and drone setpoints
        self.tip_sp_sub = rospy.Subscriber('/tooltip/setpoint_position/global', PointStamped, self.callback, tcp_nodelay=True) 

    def callback(self, tip_sp_msg):        
        p = np.asarray([tip_sp_msg.point.x, tip_sp_msg.point.y, tip_sp_msg.point.z])
        p = qv_mult(self.q, p)

        #publish PointStamped message representing target of delta-manipulator relative to base
        pos_msg = PointStamped()
        pos_msg.header.frame_id = "manipulator"
        pos_msg.header.stamp = rospy.Time.now()
        pos_msg.point.x = p[0]
        pos_msg.point.y = p[1]
        pos_msg.point.z = p[2]
        self.pub_tip_pos.publish(pos_msg) 

def transform_msg(header, child, tx, ty, tz, rx, ry, rz, rw): #function populates transformStamped message
    t =  TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = header
    t.child_frame_id = child
    t.transform.translation.x = tx
    t.transform.translation.y = ty
    t.transform.translation.z = tz
    t.transform.rotation.x = rx
    t.transform.rotation.y = ry
    t.transform.rotation.z = rz
    t.transform.rotation.w = rw
    return t

if __name__ == '__main__': #initialise node
    rospy.init_node('delta_stabilisation_node', anonymous=True)
    c = Controller()
    rospy.spin()


