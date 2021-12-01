#! /usr/bin/env python

import rospy
import numpy as np
import message_filters
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from dynamic_reconfigure.server import Server
from delta_manipulator.cfg import MountingConfig

class cfg:
    def __init__(self, nozzle, base_offset, delta_init):
        self.nozzle = nozzle
        self.base_offset = base_offset
        self.delta_init = delta_init
    @staticmethod
    def config_callback(config, level): 
        cfg.nozzle = np.asarray([0.0, 0.0, -config.nozzle])
        cfg.base_offset = np.asarray([0.0, -config.drone2basey, config.drone2basez])
        cfg.delta_init = np.array([0.0, 0.0, -config.base2tip])
        return config    

class quat:
    def __init__(self):
        pass
    #multiply 2 quaternions (only used within qv_mult)
    def q_mult(self, q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
        z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
        return np.asarray([w, x, y, z])

    #find conjugate of quaternion (also only used within qv_mult)
    def q_conjugate(self, q):
        w, x, y, z = q
        return np.asarray([w, -x, -y, -z])

    #multiply quaternion and vector (i.e perform a rotation on a vector)
    def qv_mult(self, q1, v1):
        q2 = [0.0, v1[0], v1[1], v1[2]]
        return np.asarray(self.q_mult(self.q_mult(q1, q2), self.q_conjugate(q1))[1:])

class Stabilize:
    def __init__(self, sub_drone_pose, sub_drone_setpoint):
        position, orientation = PoseStampedMsg().read(sub_drone_pose)
        position_sp, orientation_sp = PoseStampedMsg().read(sub_drone_setpoint)
        self.p = position - position_sp
        self.q = quat.q_mult(orientation,quat.q_conjugate(orientation_sp))
        self.qinv = quat.q_conjugate(self.q)
    
    def callback(self):
        delta_target = cfg.base_offset + cfg.delta_init + cfg.nozzle - quat.qv_mult(self.q,cfg.base_offset+cfg.nozzle) - self.p 
        delta_target = np.asarray(quat.qv_mult(self.qinv,delta_target))
        pos_msg = PointStampedMsg().write(delta_target[0], delta_target[1], delta_target[2])
        return pos_msg

class PointStampedMsg:
    def __init__(self):
        pass
    def write(self, x, y, z):
        self.msg = PointStamped()
        self.msg.header.frame_id = "/fcu"
        self.msg.header.stamp = rospy.Time.now()
        self.msg.point.x = x
        self.msg.point.y = y
        self.msg.point.z = z
        return self.msg

class PoseStampedMsg:
    def __init__(self):
        pass
    def read(self, msg):
        p = np.asarray([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        q = np.asarray([msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])
        return p, q

class Controller: #init publishers and subscribers
    def __init__(self):
        robot_name = rospy.get_param('/namespace') 

        self.pub_pos_tip = rospy.Publisher(robot_name+'/tip_position_local', PointStamped, queue_size=1)

        self.sub_drone_pose = message_filters.Subscriber('/mavros/local_position/pose', PoseStamped) #drone measured pose subscriber
        self.sub_drone_setpoint = message_filters.Subscriber('/mavros/setpoint_position/local', PoseStamped) #drone setpoint position subscriber
    
    def loop(self):
        ts = message_filters.ApproximateTimeSynchronizer([self.sub_drone_pose, self.sub_drone_setpoint], 1, 100)
        ts.registerCallback(self.tip_callback)
        
    def tip_callback(self, sub_drone_pose, sub_drone_setpoint): #callback calculates servo angles/torques
        pos = Stabilize(sub_drone_pose, sub_drone_setpoint).callback()
        self.pub_pos_tip.publish(pos)
        
if __name__ == '__main__': #initialise node and run loop
    rospy.init_node('delta_stabilisation_node', anonymous=True)
    srv = Server(MountingConfig, cfg.config_callback)
    Controller().loop()
    rospy.spin()