#! /usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

#script converts lidar odometry to mavros pose
class Converter:
    def __init__(self):   
        #Publish pose
        self.mavros_pose_pub= rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=1, tcp_nodelay=True)
        
        #Subscribe to transform
        self.lidar_pose_sub = rospy.Subscriber('/aft_mapped_to_init', Odometry, self.callback, tcp_nodelay=True)
        
    def callback(self, lidar_pose_msg):
        #assign Odometry message to PoseStamped
        mavros_pose_msg = PoseStamped()
        mavros_pose_msg.header.frame_id = "base_link"
        mavros_pose_msg.header.stamp = rospy.Time.now()
        mavros_pose_msg.pose.position = lidar_pose_msg.pose.pose.position
        mavros_pose_msg.pose.orientation = lidar_pose_msg.pose.pose.orientation
        #publish
        self.mavros_pose_pub.publish(mavros_pose_msg)

if __name__ == '__main__': #initialise node
    rospy.init_node('Lidar_msg_converter', anonymous=True)
    c = Converter()
    rospy.spin()