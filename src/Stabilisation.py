#! /usr/bin/env python

import rospy
import numpy as np
import message_filters
from geometry_msgs.msg import PointStamped
from tf2_msgs.msg import TFMessage 
from dynamic_reconfigure.server import Server
from delta_manipulator.cfg import MountingConfig

class cfg:
    def __init__(self, nozzle, base2tip, delta_init):
        self.nozzle = nozzle
        self.base2tip = base2tip
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
        return w, x, y, z

    #find conjugate of quaternion (also only used within qv_mult)
    def q_conjugate(self, q):
        w, x, y, z = q
        return w, -x, -y, -z

    #multiply quaternion and vector (i.e perform a rotation on a vector)
    def qv_mult(self, q1, v1):
        q2 = [0.0, v1[0], v1[1], v1[2]]
        return np.asarray(self.q_mult(self.q_mult(q1, q2), self.q_conjugate(q1))[1:])

class Stabilize:
    def __init__(self, sub_pos_tip, sub_tf_drone):
        self.p = np.asarray([sub_tf_drone.transforms[0].transform.translation.y, #note that x/y/z axes are flipped/reassigned to match delta-arm conventions
            -sub_tf_drone.transforms[0].transform.translation.z,
            -sub_tf_drone.transforms[0].transform.translation.x])

        self.q = np.asarray([sub_tf_drone.transforms[0].transform.rotation.w,
            sub_tf_drone.transforms[0].transform.rotation.y,
            -sub_tf_drone.transforms[0].transform.rotation.z,
            -sub_tf_drone.transforms[0].transform.rotation.x])
        
        self.qinv = quat.q_conjugate(self.q)

        self.delta_global_pos = np.asarray([sub_pos_tip.point.x, sub_pos_tip.point.y, sub_pos_tip.point.z]) + cfg.delta_init
    
    def callback(self):
        delta_target = np.asarray(quat.qv_mult(self.q,cfg.base_offset)) - cfg.base_offset - self.p + self.delta_global_pos - np.asarray(quat.qv_mult(self.q,cfg.nozzle)) + cfg.nozzle
        delta_target = np.asarray(quat.qv_mult(self.qinv,delta_target))
        pos_msg = PointStampedMsg(delta_target[0], delta_target[1], delta_target[2]).msg
        return pos_msg

class PointStampedMsg:
    def __init__(self, x, y, z):
        self.msg = PointStamped()
        self.msg.header.frame_id = "/fcu"
        self.msg.header.stamp = rospy.Time.now()
        self.msg.point.x = x
        self.msg.point.y = y
        self.msg.point.z = z


class Controller: #init publishers and subscribers
    def __init__(self):
        robot_name = rospy.get_param('/namespace') 

        self.pub_pos_tip = rospy.Publisher(robot_name+'/tip_position_local', PointStamped, queue_size=1)

        self.sub_pos_tip = message_filters.Subscriber(robot_name+'/tip_position_world', PointStamped) #drone trajectory subscriber
        self.sub_tf_drone = message_filters.Subscriber(robot_name+'/drone_position_world', TFMessage) #drone true tf subscriber
    
    def loop(self):
        ts = message_filters.ApproximateTimeSynchronizer([self.sub_pos_tip, self.sub_pos_drone], 1, 100)
        ts.registerCallback(self.tip_callback)
        
    def tip_callback(self, sub_pos_tip, sub_tf_drone): #callback calculates servo angles/torques
        pos = Stabilize(sub_pos_tip, sub_tf_drone).callback()
        self.pub_pos_tip.publish(pos)
        
if __name__ == '__main__': #initialise node and run loop
    rospy.init_node('delta_stabilisation_node', anonymous=True)
    srv = Server(MountingConfig, cfg.config_callback)
    Controller().loop()
    rospy.spin()