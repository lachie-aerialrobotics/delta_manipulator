#! /usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
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
        index = 0
        manip_path_msg = Path()
        manip_path_msg.header.stamp = path_msg.header.stamp
        manip_path_msg.header.frame_id = path_msg.header.frame_id
        while index < len(path_msg.poses):
            if index == 0:
                rospy.loginfo("Starting trajectory...")
            new_position = PoseStamped()
            new_position.header.frame_id = path_msg.header.frame_id
            new_position.header.stamp = path_msg.header.stamp
            new_position.pose.position.x = path_msg.poses[index].pose.position.x + self.tip_init_x 
            new_position.pose.position.y = path_msg.poses[index].pose.position.y + self.tip_init_y
            new_position.pose.position.z = path_msg.poses[index].pose.position.z + self.tip_init_z -0.17125  
            manip_path_msg.poses.append(new_position)
            index += 1
        rospy.loginfo("Completed computing delta trajectory!")
        self.manip_path_pub.publish(manip_path_msg)

if __name__ == '__main__': #initialise node
    rospy.init_node('tooltip_path_node', anonymous=True)
    s = Setpoint()
    rospy.spin()





