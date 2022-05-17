#! /usr/bin/env python
import rospy
import numpy as np
from std_msgs import int, bool

from dynamic_reconfigure.server import Server
from delta_manipulator.cfg import ExtruderConfig

def config_callback(config, level): 
    rospy.loginfo("Updating motor command")

    resistance_msg = config.resistance
    motor_msg = config.isMotorOn

    resistance_pub.publish(resistance_msg)
    motor_pub.publish(motor_msg)

    return config

if __name__ == '__main__': #initialise node
    rospy.init_node('extruder_controller', anonymous=True)
    resistance_pub = rospy.Publisher('/resistance', int, queue_size=1, tcp_nodelay=True)
    motor_pub = rospy.Publisher('/motor_on', bool, queue_size=1, tcp_nodelay=True)
    srv = Server(ExtruderConfig, config_callback)
    rospy.spin()