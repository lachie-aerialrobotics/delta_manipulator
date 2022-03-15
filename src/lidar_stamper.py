#! /usr/bin/env python
import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2

class Converter:
    def __init__(self):   
        #Publish pose
        self.mavros_pose_pub= rospy.Publisher('/os_cloud_node/points_stamped', PointCloud2, queue_size=1, tcp_nodelay=True)
        
        #Subscribe to transform
        self.lidar_pose_sub = rospy.Subscriber('/os_cloud_node/points', PointCloud2, self.callback, tcp_nodelay=True)
        
    def callback(self, lidar_point_msg):
        lidar_point_msg_stamped = lidar_point_msg
        lidar_point_msg_stamped.header.stamp = rospy.Time.now()

        self.lidar_pose_pub.publish(lidar_point_msg_stamped)

if __name__ == '__main__': #initialise node
    rospy.init_node('Lidar_msg_stamper', anonymous=True)
    c = Converter()
    rospy.spin()