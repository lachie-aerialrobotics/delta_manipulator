#! /usr/bin/env python
import rospy
import numpy as np
import message_filters
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from dynamic_reconfigure.server import Server
from delta_manipulator.cfg import MountingConfig

class cfg: #class to store variables from dynamic reconfigure
    def __init__(self, nozzle, base_offset, delta_init):
        self.nozzle = nozzle
        self.base_offset = base_offset
        self.delta_init = delta_init

    @staticmethod
    def config_callback(config, level): 
        cfg.nozzle = np.asarray([config.nozzle, 0.0, 0.0]) #in drone coordinates
        cfg.base_offset = np.asarray([config.drone2basex, 0.0, config.drone2basez]) #in drone coordinates
        cfg.delta_init = np.array([config.base2tipx, config.base2tipy, config.base2tipz]) #in world coordinates
        return config    

class quat: #the methods in this class are useful quaternion/vector operations
    def __init__(self):
        pass
    
    def q_mult(self, q1, q2): #multiply 2 quaternions (only used within qv_mult)
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
        z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
        return np.asarray([w, x, y, z])

    def q_conjugate(self, q): #find conjugate of quaternion (also only used within qv_mult)
        return np.asarray([q[0], -q[1], -q[2], -q[3]])
 
    def qv_mult(self, q1, v1):#multiply quaternion and vector (i.e perform a rotation on a vector)
        q2 = [0.0, v1[0], v1[1], v1[2]]
        return np.asarray(self.q_mult(self.q_mult(q1, q2), self.q_conjugate(q1))[1:])

class Stabilize:
    def __init__(self, sub_drone_pose, sub_drone_setpoint):
        position, orientation = PoseStampedMsg().read(sub_drone_pose)
        position_sp, orientation_sp = PoseStampedMsg().read(sub_drone_setpoint)
        self.p = position - position_sp #error position vector in drone coordinates
        self.q = orientation #temporary simplification
        #orientation_sp_inv = quat().q_conjugate(orientation_sp)
        #self.q = quat().q_mult(orientation, orientation_sp_inv) #error quaternion in drone coordinates
        self.qinv = quat().q_conjugate(self.q)
    
    def callback(self):
        delta_target = cfg.base_offset + cfg.delta_init + cfg.nozzle - quat().qv_mult(self.q,cfg.base_offset+cfg.nozzle) - self.p #delta target in drone coordinates
        delta_target = quat().qv_mult(self.qinv,delta_target) #delta target in drone body coordinates
        delta_target_drone_x = -delta_target[1]
        delta_target_drone_y = delta_target[2]
        delta_target_drone_z = -delta_target[0]
        pos_msg = PointStampedMsg().write(delta_target_drone_x, delta_target_drone_y, delta_target_drone_z)
        return pos_msg

class PointStampedMsg:
    def __init__(self):
        pass
    def write(self, x, y, z):
        msg = PointStamped()
        msg.header.frame_id = "/fcu"
        msg.header.stamp = rospy.Time.now()
        msg.point.x = x
        msg.point.y = y
        msg.point.z = z
        return msg

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
        self.pub_pos_tip = rospy.Publisher(robot_name+'/tip_position_local', PointStamped, queue_size=1) #delta target position publisher
        self.sub_drone_pose = message_filters.Subscriber('/mavros/local_position/pose', PoseStamped) #drone measured pose subscriber
        self.sub_drone_setpoint = message_filters.Subscriber('/mavros/setpoint_position/local', PoseStamped) #drone setpoint position subscriber
    
    def loop(self):
    #     ts = message_filters.ApproximateTimeSynchronizer([self.sub_drone_pose, self.sub_drone_setpoint], 1, 100)
    #     ts.registerCallback(self.tip_callback)
        
    # def tip_callback(self, sub_drone_pose, sub_drone_setpoint): #callback calculates servo angles/torques
        pos = Stabilize(self.sub_drone_pose, self.sub_drone_setpoint).callback()
        self.pub_pos_tip.publish(pos)
        
if __name__ == '__main__': #initialise node and run loop
    rospy.init_node('delta_stabilisation_node', anonymous=True)
    srv = Server(MountingConfig, cfg.config_callback)
    Controller().loop()
    rospy.spin()