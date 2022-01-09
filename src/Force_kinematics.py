#!/usr/bin/env python
from Servo_writer import config_callback
import rospy
import numpy as np
from geometry_msgs.msg import PointStamped
from delta_manipulator.msg import servo_angles
from dynamic_reconfigure.server import Server
from delta_manipulator.cfg import TorqueConfig

class cfg:
    torque2current_gradient = rospy.get_param('manipulator/servo/ctrl_table/torque2current_gradient')
    torque2current_intercept = rospy.get_param('manipulator/servo/ctrl_table/torque2current_intercept')
    torque2current_unit = rospy.get_param('manipulator/servo/ctrl_table/torque2current_unit')

def config_callback(config,level):
    I1 = torque2current(config.T1)
    I2 = torque2current(config.T2)
    I3 = torque2current(config.T3)

    msg = servo_angles()
    msg.header.stamp = rospy.Time.now() 
    msg.header.frame_id = "servos"
    msg.theta1 = I1
    msg.theta2 = I2
    msg.theta3 = I3

    pub_crrnt.publish(msg)
    return config

def torque2current(T):
    #calculate servo current limit to achieve desired torque (taken from datasheet graph)
    I = int((cfg.torque2current_gradient * abs(T) + cfg.torque2current_intercept) *1000 / cfg.torque2current_unit)
    if I > 2047:
        I = 2047 #do not let current exceed limit
    return I

if __name__ == '__main__': #initialise node and run loop
    rospy.init_node('delta_force_kinematics', anonymous=True)
    pub_crrnt = rospy.Publisher('/servo/current_limits', servo_angles, queue_size=1) # servo current publisher
    srv = Server(TorqueConfig, config_callback)
    rospy.spin()