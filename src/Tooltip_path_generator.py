#! /usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import Path

class Setpoint:
    def __init__(self):   
        #retrieve params from parameter server
        self.rate = rospy.get_param('/rate')
        self.tip_init_x = rospy.get_param('/manipulator/tooltip/tip_init_x')
        self.tip_init_y = rospy.get_param('/manipulator/tooltip/tip_init_y')
        self.tip_init_z = rospy.get_param('/manipulator/tooltip/tip_init_z')
    
        #Publish target positions for drone and manipulator tooltip
        self.manip_path_pub = rospy.Publisher('/path/tooltip', Path, queue_size=1, tcp_nodelay=True)

        #Subscribe to joystick inputs
        self.drone_path_sub = rospy.Subscriber('/path/drone', Path, self.path_callback, tcp_nodelay=True)

    def path_callback(self, path_msg):
        self.index = 0
        while self.index < len(self.path_msg.pose):
            if self.index == 0:
                rospy.loginfo("Starting trajectory...")
            self.manip_path_msg = Path()
            self.manip_path_msg.poses[self.index].pose.position.x = self.path_msg.poses[self.index].pose.position.x + self.tip_init_x 
            self.manip_path_msg.poses[self.index].pose.position.y = self.path_msg.poses[self.index].pose.position.y + self.tip_init_y
            self.manip_path_msg.poses[self.index].pose.position.z = self.path_msg.poses[self.index].pose.position.z + self.tip_init_z
            self.index += 1
        rospy.loginfo("Completed computing delta trajectory!")
        self.manip_path_pub.publish(self.manip_path_msg)

if __name__ == '__main__': #initialise node
    rospy.init_node('joystick_node', anonymous=True)
    s = Setpoint()
    rospy.spin()





