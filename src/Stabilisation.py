#! /usr/bin/env python
import rospy
import numpy as np
import message_filters
import tf2_ros
from tf.transformations import quaternion_from_euler
from quaternion_functions import q_conjugate, qv_mult, q_mult
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped

from dynamic_reconfigure.server import Server
from delta_manipulator.cfg import StabilisationConfig

#script calculates required end-effector position of manipulator to stabilise base perturbations using tf2 frames
class Controller:
    def __init__(self):
        #get params from parameter server
        self.rate = rospy.get_param('/rate')
        nozzlex = rospy.get_param('/manipulator/geometry/nozzle')
        drone2basex = rospy.get_param('/manipulator/geometry/drone2base_x')  
        drone2basez = rospy.get_param('/manipulator/geometry/drone2base_z')   
        base_roll = rospy.get_param('/manipulator/geometry/base_roll')
        base_pitch = rospy.get_param('/manipulator/geometry/base_pitch')
        base_yaw = rospy.get_param('/manipulator/geometry/base_yaw')
        self.retracted_x = rospy.get_param('/manipulator/tooltip/tip_retracted_x')
        self.retracted_y = rospy.get_param('/manipulator/tooltip/tip_retracted_y')
        self.retracted_z = rospy.get_param('/manipulator/tooltip/tip_retracted_z')

        #initialise variables
        self.p_tooltip = np.asarray([0.0, 0.0, -nozzlex])
        self.p_fcu2base = np.asarray([drone2basex, 0.0, drone2basez])
        self.q_fcu2base = quaternion_from_euler(np.deg2rad(base_pitch) , np.deg2rad(base_roll), np.deg2rad(base_yaw))
        self.p_base2plat = self.p_fcu2base + self.p_tooltip

        #broadcast static frames
        br_static = tf2_ros.StaticTransformBroadcaster()
        tf_fcu2base = transform_msg(
            "base_link", "manipulator",
            self.p_fcu2base[0], self.p_fcu2base[1], self.p_fcu2base[2],
            self.q_fcu2base[0], self.q_fcu2base[1], self.q_fcu2base[2], self.q_fcu2base[3]
        )

        tf_platform2tooltip = transform_msg(
            "platform", "tooltip",
            self.p_tooltip[0], self.p_tooltip[1], self.p_tooltip[2],
            0.0, 0.0, 0.0, 1.0
        )    
        br_static.sendTransform([tf_fcu2base, tf_platform2tooltip])

        #init dynamic reconfigure server
        self.srv = Server(StabilisationConfig, config_callback)

        #publish platform setpoint
        self.pub_tip_pos = rospy.Publisher('/tooltip/setpoint_position/local', PointStamped, queue_size=1, tcp_nodelay=True) 

        #subscribe to tooltip and drone setpoints
        tip_sp_sub = message_filters.Subscriber('/tooltip/setpoint_position/global', PointStamped, tcp_nodelay=True) 
        drone_pose_sub = message_filters.Subscriber('/mavros/local_position/pose', PoseStamped, tcp_nodelay=True)
        drone_sp_sub = message_filters.Subscriber('/mavros/setpoint_position/local', PoseStamped, tcp_nodelay=True)
        ts = message_filters.ApproximateTimeSynchronizer([tip_sp_sub, drone_pose_sub, drone_sp_sub], 1, 0.1)
        ts.registerCallback(self.callback)

    def callback(self, tip_sp_msg, drone_pose_msg, drone_sp_msg):
        #vector stuff to work out location of end effector, there may be an easier way to do this using the tf2 transforms
        #already being broadcasted, but I couldn't work it out :p

        if self.retract == True:
            sp_p_base2plat = np.asarray([self.retracted_x, self.retracted_y, self.retracted_z])
        elif self.retract == False:                                                                                                                       
            if self.stabilise == True:
                q_fcu = np.asarray([drone_pose_msg.pose.orientation.x,
                                    drone_pose_msg.pose.orientation.y,
                                    drone_pose_msg.pose.orientation.z,
                                    drone_pose_msg.pose.orientation.w])

                p_fcu = np.asarray([drone_pose_msg.pose.position.x,
                                    drone_pose_msg.pose.position.y,
                                    drone_pose_msg.pose.position.z])
            elif self.stabilise == False:
                q_fcu = np.asarray([drone_sp_msg.pose.orientation.x,
                                    drone_sp_msg.pose.orientation.y,
                                    drone_sp_msg.pose.orientation.z,
                                    drone_sp_msg.pose.orientation.w])

                p_fcu = np.asarray([drone_sp_msg.pose.position.x,
                                    drone_sp_msg.pose.position.y,
                                    drone_sp_msg.pose.position.z])            

            self.q_base = q_mult(q_fcu, self.q_fcu2base)
            self.q_base_conj = np.asarray(q_conjugate(self.q_base))
            self.p_base = p_fcu + qv_mult(q_fcu, self.p_fcu2base)
            p_tip_sp = np.asarray([tip_sp_msg.point.x, tip_sp_msg.point.y, tip_sp_msg.point.z])
            sp_p_base2plat = qv_mult(self.q_base_conj, (-self.p_base + p_tip_sp)) - self.p_tooltip

        sp_p_base2plat = self.speed_limiter(self.p_base2plat, sp_p_base2plat)
        self.p_base2plat = sp_p_base2plat
        
        #publish PointStamped message representing target of delta-manipulator relative to base
        pos_msg = PointStamped()
        pos_msg.header.frame_id = "manipulator"
        pos_msg.header.stamp = rospy.Time.now()
        pos_msg.point.x = self.p_base2plat[0]
        pos_msg.point.y = self.p_base2plat[1]
        pos_msg.point.z = self.p_base2plat[2]
        self.pub_tip_pos.publish(pos_msg) 

    def speed_limiter(self, p, p_sp):
        p_diff = p_sp - p
        p_diff_norm = np.linalg.norm(p_diff)
        if p_diff_norm != 0.0:
            p_diff_dir = p_diff / p_diff_norm
            v_step = self.v_max/self.rate
            if np.linalg.norm(p_sp - p) > v_step:
                p_sp = p + p_diff_dir * v_step
        return p_sp  

def config_callback(config, level): 
    Controller.v_max = config.v_max_tooltip
    Controller.stabilise = config.stabilise
    Controller.retract = config.retract
    return config


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