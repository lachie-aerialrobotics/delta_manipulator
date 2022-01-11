#! /usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped

#script converts vicon tf to mavros pose
class Converter:
    def __init__(self):   
        #Publish pose
        self.mavros_pose_pub = rospy.Publisher('/hexacopter/vrpn_client/raw_transform', TransformStamped, queue_size=1, tcp_nodelay=True)
        #Subscribe to transform
        self.vicon_tf_sub = rospy.Subscriber('/mavros/vision_pose/pose', PoseStamped, self.callback, tcp_nodelay=True)

    def callback(self, vicon_tf_msg):
        #assign TransformStamped message to PoseStamped
        mavros_pose_msg = PoseStamped()
        mavros_pose_msg.header.frame_id = "base_link"
        mavros_pose_msg.header.stamp = rospy.Time.now()
        mavros_pose_msg.pose.position.x = vicon_tf_msg.transform.translation.x
        mavros_pose_msg.pose.position.y = vicon_tf_msg.transform.translation.y
        mavros_pose_msg.pose.position.z = vicon_tf_msg.transform.translation.z
        mavros_pose_msg.pose.orientation.x = vicon_tf_msg.transform.rotation.x
        mavros_pose_msg.pose.orientation.y = vicon_tf_msg.transform.rotation.y
        mavros_pose_msg.pose.orientation.z = vicon_tf_msg.transform.rotation.z
        mavros_pose_msg.pose.orientation.w = vicon_tf_msg.transform.rotation.w

        #publish
        self.mavros_pose_pub.publish(mavros_pose_msg)

if __name__ == '__main__': #initialise node
    rospy.init_node('joystick_node', anonymous=True)
    c = Converter()
    rospy.spin()