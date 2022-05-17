#! /usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import UInt16, Bool

from dynamic_reconfigure.server import Server
from delta_manipulator.cfg import ExtruderConfig

def config_callback(config, level): 
    rospy.loginfo("Updating extruder command")

    resistance_msg = UInt16()
    motor_msg = Bool()

    resistance_msg.data = config.resistance
    motor_msg.data = config.isMotorOn

    resistance_pub.publish(resistance_msg)
    motor_pub.publish(motor_msg)

    return config

if __name__ == '__main__': #initialise node
    rospy.init_node('extruder_controller', anonymous=True)
    resistance_pub = rospy.Publisher('/resistance', UInt16, queue_size=1, tcp_nodelay=True)
    motor_pub = rospy.Publisher('/motor_on', Bool, queue_size=1, tcp_nodelay=True)
    srv = Server(ExtruderConfig, config_callback)
    rospy.spin()