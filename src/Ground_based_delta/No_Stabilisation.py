#! /usr/bin/env python
import rospy
import numpy as np
import message_filters
from tf2_geometry_msgs import do_transform_point
import tf2_ros
from tf2_geometry_msgs import PointStamped
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped

#script calculates required end-effector position of manipulator to stabilise base perturbations using tf2 frames
class Controller:
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)

        #broadcast static frames
        br_static = tf2_ros.StaticTransformBroadcaster()
        tf_base2manip = transform_msg(
            "base_link", "manipulator",
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0
        )
        q = quaternion_from_euler(np.pi, 0.0, 0.0)
        tf_map2base = transform_msg(
            "map", "base_link",
            0.0, 0.0, 0.0,
            q[0], q[1], q[2], q[3]
        )
  
        br_static.sendTransform([tf_map2base, tf_base2manip])

        #publish platform setpoint
        self.pub_tip_pos = rospy.Publisher('/tooltip/setpoint_position/local', PointStamped, queue_size=1, tcp_nodelay=True) 

        #subscribe to tooltip and drone setpoints
        self.tip_sp_sub = rospy.Subscriber('/tooltip/setpoint_position/global', PointStamped, self.callback, tcp_nodelay=True) 

    def callback(self, tip_sp_msg):        
        trans = self.tfBuffer.lookup_transform('map', 'manipulator', rospy.Time(), rospy.Duration(0.5))
        do_transform_point(tip_sp_msg, trans)
        #publish PointStamped message representing target of delta-manipulator relative to base
        pos_msg = PointStamped()
        pos_msg.header.frame_id = "manipulator"
        pos_msg.header.stamp = rospy.Time.now()
        pos_msg.point.x = tip_sp_msg.point.x
        pos_msg.point.y = tip_sp_msg.point.y
        pos_msg.point.z = tip_sp_msg.point.z
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


