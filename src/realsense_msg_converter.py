#! /usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped

#script converts vicon tf to mavros pose
class Converter:
    def __init__(self):   
        rate = rospy.get_param('/rate')

        #Publish pose
        self.mavros_pose_pub= rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=1, tcp_nodelay=True)
        
        #Subscribe to transform
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)
        

        rospy.Timer(rospy.Duration(1.0/rate), self.callback)

    def callback(self, event):
        #assign TransformStamped message to PoseStamped
        try:
            
            self.trans = self.tfBuffer.lookup_transform('map', 'base_link',  rospy.Time())
        
            mavros_pose_msg = PoseStamped()
            mavros_pose_msg.header.frame_id = "base_link"
            mavros_pose_msg.header.stamp = rospy.Time.now()
            mavros_pose_msg.pose.position.x = self.trans.transform.translation.x
            mavros_pose_msg.pose.position.y = self.trans.transform.translation.y
            mavros_pose_msg.pose.position.z = self.trans.transform.translation.z
            mavros_pose_msg.pose.orientation.x = self.trans.transform.rotation.x
            mavros_pose_msg.pose.orientation.y = self.trans.transform.rotation.y
            mavros_pose_msg.pose.orientation.z = self.trans.transform.rotation.z
            mavros_pose_msg.pose.orientation.w = self.trans.transform.rotation.w
            
            #publish
            self.mavros_pose_pub.publish(mavros_pose_msg)
            rospy.loginfo_once("tf published :)")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn_once("Waiting for tf data from realsense")

if __name__ == '__main__': #initialise node
    rospy.init_node('realsense_msg_converter', anonymous=True)
    c = Converter()
    rospy.spin()