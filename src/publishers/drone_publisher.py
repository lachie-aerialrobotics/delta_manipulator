#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped

class Trajectory:
    ros_rate = 100
    def __init__(self):
        self.pub_pos = rospy.Publisher('/mavros/local_position/pose', PoseStamped, queue_size=1)     
        rate = rospy.Rate(self.ros_rate) # in Hz  
        while not rospy.is_shutdown():
            x = 0.0
            y = 0.0
            z = 0.0
            yaw = 0.0
            pos = callback_pos().write(x,y,z,yaw)
            self.pub_pos.publish(pos)
            rate.sleep()

class callback_pos: 
    def __init__(self):
        pass
    def write(self,x,y,z,yaw):
        self.pos_msg = PoseStamped()
        self.pos_msg.header.frame_id = "/tf"
        self.pos_msg.header.stamp = rospy.Time.now()

        self.pos_msg.pose.orientation.w = np.cos(yaw/2)
        self.pos_msg.pose.orientation.x = 0.0
        self.pos_msg.pose.orientation.y = 0.0
        self.pos_msg.pose.orientation.z = np.sin(yaw/2)

        self.pos_msg.pose.position.x = x
        self.pos_msg.pose.position.y = y
        self.pos_msg.pose.position.z = z
        return self.pos_msg
         
if __name__ == '__main__':    
    rospy.init_node('talker_target', anonymous=True)    
    try:
        Trajectory()
    except rospy.ROSInterruptException:
        pass