#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped

class Trajectory:
    ros_rate = 100
    def __init__(self):
        robot_name = rospy.get_param('/namespace') 
        self.pub_pos = rospy.Publisher('/mavros/local_position/pose', PoseStamped, queue_size=1) 
        self.pub_pos_sp = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)   
        self.pub_tip_sp = rospy.Publisher(robot_name+'/tip/setpoint_position', PoseStamped, queue_size=1) 
        self.pub_tip_force = rospy.Publisher(robot_name+'/tip/force', PointStamped, queue_size=1) 
        rate = rospy.Rate(self.ros_rate) # in Hz  
        
        while not rospy.is_shutdown():
            x = 0.0
            y = 0.0
            z = 1.0
            yaw = np.pi/16
            pos = callback_pos().write(x,y,z,yaw)
            pos_sp = callback_pos().write(x,y,z,yaw)
            tip_sp = callback_pos().write(x+0.8,y,z,yaw)
            self.pub_pos.publish(pos)
            self.pub_pos_sp.publish(pos_sp)
            self.pub_tip_sp.publish(tip_sp)

            force_msg = PointStamped()
            force_msg.header.frame_id = "servos"
            force_msg.header.stamp = rospy.Time.now()
            force_msg.point.x = 200
            force_msg.point.y = 200
            force_msg.point.z = 200
            self.pub_tip_force.publish(force_msg)

            rate.sleep()

class callback_pos: 
    def __init__(self):
        pass
    def write(self,x,y,z,yaw):
        self.pos_msg = PoseStamped()
        self.pos_msg.header.frame_id = "map"
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