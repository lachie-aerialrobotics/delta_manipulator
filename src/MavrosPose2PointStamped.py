#! /usr/bin/env python
import rospy
import numpy as np
import message_filters
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped  

def callback(target_msg):
    pos_msg = PointStampedMsg().write(target_msg.pose.position.x, target_msg.pose.position.y, target_msg.pose.position.z)
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

class Controller: #init publishers and subscribers
    def __init__(self):
        robot_name = rospy.get_param('/namespace') 
        self.pub_pos_tip = rospy.Publisher(robot_name+'/tip_position_local', PointStamped, queue_size=1) #delta target position publisher
        self.sub_drone_setpoint = message_filters.Subscriber('/mavros/setpoint_position/local', PoseStamped) #drone setpoint position subscriber
    
    def loop(self):
        ts = message_filters.ApproximateTimeSynchronizer([self.sub_drone_setpoint], 1, 100)
        ts.registerCallback(self.tip_callback)
        
    def tip_callback(self, sub_drone_setpoint): #callback calculates servo angles/torques
        pos = callback(sub_drone_setpoint)
        self.pub_pos_tip.publish(pos)
        
if __name__ == '__main__': #initialise node and run loop
    rospy.init_node('delta_stabilisation_node', anonymous=True)
    Controller().loop()
    rospy.spin()