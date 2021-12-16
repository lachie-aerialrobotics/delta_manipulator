#! /usr/bin/env python
import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped

#script calculates required end-effector position of manipulator to stabilise base perturbations using tf2 frames
class Controller():
    def __init__(self): #init publishers and subscribers
        self.pub_tip_pos = rospy.Publisher(robot_name+'/tip/local_position', PointStamped, queue_size=1) #delta target position publisher
        self.tip_sp_sub = rospy.Subscriber(robot_name+'/tip/setpoint_position', PoseStamped, self.callback) #drone measured pose subscriber

    def callback(self, tip_sp_msg):
        #lookup required transforms
        tf_base2map = tfBuffer.lookup_transform('base','map', rospy.Time.now(), rospy.Duration(rospy.get_param('/tf_buffer_wait')))
        tf_tip2plat = tfBuffer.lookup_transform('tooltip','platform', rospy.Time.now(), rospy.Duration(rospy.get_param('/tf_buffer_wait')))

        #perform successive transforms to get delta-arm target in base coordinates (and including offset for nozzle)
        self.target = tf2_geometry_msgs.do_transform_pose(tip_sp_msg, tf_base2map)
        self.target = tf2_geometry_msgs.do_transform_pose(self.target, tf_tip2plat)   

        #publish PointStamped message representing target of delta-manipulator relative to base
        pos_msg = PointStamped()
        pos_msg.header.frame_id = "base"
        pos_msg.header.stamp = rospy.Time.now()
        pos_msg.point.x = self.target.pose.position.x
        pos_msg.point.y = self.target.pose.position.y
        pos_msg.point.z = self.target.pose.position.z
        self.pub_tip_pos.publish(pos_msg) 

        br_base2platform = tf2_ros.TransformBroadcaster()
        tf_base2platform = transform_msg(
            "base", "platform",
            pos_msg.point.x, pos_msg.point.y, pos_msg.point.z,
            0.0, 0.0, 0.0, 1.0
        )
        br_base2platform.sendTransform(tf_base2platform)

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
    robot_name = rospy.get_param('/namespace')
    #rospy.sleep(rospy.get_param('/tf_wait')) #sleep gives time for tf frames to initialise and prevents non-fatal (but ugly) time_sync error
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    c = Controller()
    rospy.spin()


