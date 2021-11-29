#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import PointStamped
from dynamic_reconfigure.server import Server
from delta_manipulator.cfg import TrajectoryConfig

class Trajectory:
    ros_rate = 100
    
    def __init__(self):
        robot_name = rospy.get_param('/namespace')
        srv = Server(TrajectoryConfig, config_callback)
        self.pub_pos = rospy.Publisher(robot_name+'/tip_position_world', PointStamped, queue_size=1)
        self.pub_force = rospy.Publisher(robot_name+'/tip_force', PointStamped, queue_size=1)
        rate = rospy.Rate(self.ros_rate) # in Hz
        
        while not rospy.is_shutdown():
            pos = callback_pos().pos_msg
            force = callback_force().force_msg
            self.pub_pos.publish(pos)
            self.pub_force.publish(force)
            rate.sleep()

class callback_pos: 
    def __init__(self):
        self.pos_msg = PointStamped()
        self.pos_msg.header.frame_id = "/world"
        self.pos_msg.header.stamp = rospy.Time.now()
        global t
        t = t + cfg.v / (cfg.r * Trajectory.ros_rate)

        if cfg.mode == 0: #static
            self.pos_msg.point.x = cfg.x
            self.pos_msg.point.y = cfg.y
            self.pos_msg.point.z = cfg.z
        if cfg.mode == 1: #line z
            self.pos_msg.point.y = cfg.x
            self.pos_msg.point.x = cfg.y
            self.pos_msg.point.z = cfg.z + cfg.r * np.cos(t)
        if cfg.mode == 2: #line y
            self.pos_msg.point.z = cfg.z
            self.pos_msg.point.x = cfg.x
            self.pos_msg.point.y = cfg.y + cfg.r * np.cos(t)
        if cfg.mode == 3: #line x
            self.pos_msg.point.y = cfg.y
            self.pos_msg.point.z = cfg.z
            self.pos_msg.point.x = cfg.x + cfg.r * np.cos(t)
        if cfg.mode == 4: #circlexy
            self.pos_msg.point.x = cfg.x + cfg.r * np.cos(t)
            self.pos_msg.point.y = cfg.y + cfg.r * np.sin(t)
            self.pos_msg.point.z = cfg.z 
        if cfg.mode == 5: #circleyz
            self.pos_msg.point.z = cfg.z + cfg.r * np.cos(t)
            self.pos_msg.point.y = cfg.y + cfg.r * np.sin(t)
            self.pos_msg.point.x = cfg.x 

class callback_force:
    def __init__(self):
        self.force_msg = PointStamped()
        self.force_msg.header.frame_id = "/fcu"
        self.force_msg.header.stamp = rospy.Time.now()
        self.force_msg.point.x = cfg.T1
        self.force_msg.point.y = cfg.T2
        self.force_msg.point.z = cfg.T3

class cfg:
    def __init__(self, r, x, y, z, v, T1, T2, T3, mode):
        self.r = r
        self.x = x
        self.y = y
        self.z = z
        self.v = v
        self.T1 = T1
        self.T2 = T2
        self.T3 = T3
        self.mode = mode

def config_callback(config, level): 
    cfg.r = config.r
    cfg.x = config.x
    cfg.y = config.y
    cfg.z = config.z
    cfg.v = config.v  
    cfg.T1 = config.T1
    cfg.T2 = config.T2
    cfg.T3 = config.T3
    cfg.mode = config.mode
    return config
         
if __name__ == '__main__':    
    rospy.init_node('talker_target', anonymous=True)
    global t
    t = 0.0
    
    try:
        Trajectory()
    except rospy.ROSInterruptException:
        pass