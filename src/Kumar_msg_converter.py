#! /usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped

#script converts vicon tf to mavros pose
class Converter:
    def __init__(self):   
        #Publish pose
        self.mavros_pose_pub= rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=1, tcp_nodelay=True)
        
        #Subscribe to transform
        self.vicon_pose_sub = rospy.Subscriber('/vicon/hexacopter/pose', PoseStamped, self.callback, tcp_nodelay=True)
        
    def callback(self, vicon_pose_msg):
        #assign TransformStamped message to PoseStamped
        mavros_pose_msg = vicon_pose_msg
        mavros_pose_msg.header.frame_id = "base_link"

        #publish
        self.mavros_pose_pub.publish(mavros_pose_msg)

if __name__ == '__main__': #initialise node
    rospy.init_node('Vicon_msg_converter', anonymous=True)
    c = Converter()
    rospy.spin()