#! /usr/bin/env python
import rospy
import numpy as np
import message_filters
import tf2_ros
from tf.transformations import quaternion_from_euler
from quaternion_functions import q_conjugate, qv_mult, q_mult
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped

#script calculates required end-effector position of manipulator to stabilise base perturbations using tf2 frames
class Controller:
    def __init__(self):
        #get params from parameter server
        nozzlex = rospy.get_param('/manipulator/geometry/nozzle')
        drone2basex = rospy.get_param('/manipulator/geometry/drone2base_x')  
        drone2basez = rospy.get_param('/manipulator/geometry/drone2base_z')   
        base_pitch = rospy.get_param('/manipulator/geometry/base_pitch')

        #initialise variables
        self.p_tooltip = np.asarray([0.0, 0.0, -nozzlex])
        self.p_fcu2base = np.asarray([drone2basex, 0.0, drone2basez])
        self.q_fcu2base = quaternion_from_euler(np.deg2rad(base_pitch) , 0.0, -np.pi/2)

        #broadcast static frames
        br_static = tf2_ros.StaticTransformBroadcaster()
        tf_fcu2base = transform_msg(
            "base_link", "manipulator",
            self.p_fcu2base[0], self.p_fcu2base[1], self.p_fcu2base[2],
            self.q_fcu2base[0], self.q_fcu2base[1], self.q_fcu2base[2], self.q_fcu2base[3]
        )

        tf_odom2lidar = transform_msg(
            "odom", "rplidar_link",
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0
        )

        tf_fcu2odom = transform_msg(
            "base_link", "odom",
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0
        )

        tf_platform2tooltip = transform_msg(
            "platform", "tooltip",
            self.p_tooltip[0], self.p_tooltip[1], self.p_tooltip[2],
            0.0, 0.0, 0.0, 1.0
        )    
        br_static.sendTransform([tf_fcu2base, tf_platform2tooltip, tf_fcu2odom, tf_odom2lidar])

        #publish platform setpoint
        self.pub_tip_pos = rospy.Publisher('/tooltip/setpoint_position/local', PointStamped, queue_size=1, tcp_nodelay=True) 

        #subscribe to tooltip and drone setpoints
        self.tip_sp_sub = message_filters.Subscriber('/tooltip/setpoint_position/global', PointStamped, tcp_nodelay=True) 
        self.drone_pose_sub = message_filters.Subscriber('/mavros/local_position/pose', PoseStamped, tcp_nodelay=True)
        ts = message_filters.ApproximateTimeSynchronizer([self.tip_sp_sub, self.drone_pose_sub], 1, 0.1)
        ts.registerCallback(self.callback)

    def callback(self, tip_sp_msg, drone_pose_msg):
        #vector stuff to work out location of end effector, there may be an easier way to do this using the tf2 transforms
        #being broadcasted, but I couldn't work it out

        q_fcu = np.asarray([drone_pose_msg.pose.orientation.x,
                            drone_pose_msg.pose.orientation.y,
                            drone_pose_msg.pose.orientation.z,
                            drone_pose_msg.pose.orientation.w])

        p_fcu = np.asarray([drone_pose_msg.pose.position.x,
                            drone_pose_msg.pose.position.y,
                            drone_pose_msg.pose.position.z])

        self.q_base = q_mult(q_fcu, self.q_fcu2base)
        self.q_base_conj = np.asarray(q_conjugate(self.q_base))
        self.p_base = p_fcu + qv_mult(q_fcu, self.p_fcu2base)
        p_tip_sp = np.asarray([tip_sp_msg.point.x, tip_sp_msg.point.y, tip_sp_msg.point.z])
        p_base2plat = qv_mult(self.q_base_conj, (-self.p_base + p_tip_sp)) - self.p_tooltip
        
        #publish PointStamped message representing target of delta-manipulator relative to base
        pos_msg = PointStamped()
        pos_msg.header.frame_id = "manipulator"
        pos_msg.header.stamp = rospy.Time.now()
        pos_msg.point.x = p_base2plat[0]
        pos_msg.point.y = p_base2plat[1]
        pos_msg.point.z = p_base2plat[2]
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


