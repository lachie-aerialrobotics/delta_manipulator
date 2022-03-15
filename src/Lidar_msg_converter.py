#! /usr/bin/env python
import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry

#script converts lidar odometry to mavros pose
class Converter:
    def __init__(self):   
        #broadcast os_sensor2base_link
        br_static = tf2_ros.StaticTransformBroadcaster()
        lidar_offset_x = rospy.get_param('/manipulator/lidar_offset_x')
        lidar_offset_y = rospy.get_param('/manipulator/lidar_offset_y')
        lidar_offset_z = rospy.get_param('/manipulator/lidar_offset_z')
        tf_base_link2os_sensor = self.transform_msg("base_link", "os_sensor", lidar_offset_x, lidar_offset_y, lidar_offset_z, 0, 0, 0, 1)
        tf_map2camera_init = self.transform_msg("map", "camera_init", 0, 0, 0, 0, 0, 0, 1)
        tf_aft_mapped2os_sensor = self.transform_msg("aft_mapped", "os_sensor", 0, 0, 0, 0, 0, 0, 1)
        br_static.sendTransform([tf_base_link2os_sensor, tf_map2camera_init, tf_aft_mapped2os_sensor])

        #Subscribe to transform
        self.lidar_pose_sub = rospy.Subscriber('/aft_mapped_to_init', Odometry, self.callback, tcp_nodelay=True)

        #Subscribe to transform
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)

        #Publish pose
        self.mavros_pose_pub= rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=1, tcp_nodelay=True)
        
          
    def callback(self, lidar_odom_msg):

        # #broadcast map2odom
        # br = tf2_ros.TransformBroadcaster()
        # tf_odom2os_sensor = self.transform_msg(
        #     "camera_init", "os_sensor",
        #     lidar_pose_msg.pose.pose.position.x, 
        #     lidar_pose_msg.pose.pose.position.y, 
        #     lidar_pose_msg.pose.pose.position.z,
        #     lidar_pose_msg.pose.pose.orientation.x, 
        #     lidar_pose_msg.pose.pose.orientation.y, 
        #     lidar_pose_msg.pose.pose.orientation.z, 
        #     lidar_pose_msg.pose.pose.orientation.w
        # )
  
        # br.sendTransform([tf_odom2os_sensor])
        # lidar_odom_msg = Odometry()
        lidar_tf_msg = TransformStamped()
        lidar_tf_msg.header = lidar_odom_msg.header
        lidar_tf_msg.child_frame_id = lidar_odom_msg.child_frame_id
        lidar_tf_msg.transform.translation = lidar_odom_msg.pose.pose.position
        lidar_tf_msg.transform.rotation = lidar_odom_msg.pose.pose.orientation

        br = tf2_ros.TransformBroadcaster()
        br.sendTransform([lidar_tf_msg])

        try:
            tf_map2base_link = self.tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
            mavros_pose_msg = PoseStamped()
            mavros_pose_msg.header.frame_id = "base_link"
            mavros_pose_msg.header.stamp = rospy.Time.now()
            mavros_pose_msg.pose.position.x = tf_map2base_link.transform.translation.x
            mavros_pose_msg.pose.position.y = tf_map2base_link.transform.translation.y
            mavros_pose_msg.pose.position.z = tf_map2base_link.transform.translation.z
            mavros_pose_msg.pose.orientation.x = tf_map2base_link.transform.rotation.x
            mavros_pose_msg.pose.orientation.y = tf_map2base_link.transform.rotation.y
            mavros_pose_msg.pose.orientation.z = tf_map2base_link.transform.rotation.z
            mavros_pose_msg.pose.orientation.w = tf_map2base_link.transform.rotation.w
            rospy.loginfo_once("tf published :)")
            
            #publish
            self.mavros_pose_pub.publish(mavros_pose_msg)        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn_once("Waiting for tf data from lidar")

        

        # #assign TransformStamped message to PoseStamped
        # try:   
        #     self.trans = self.tfBuffer.lookup_transform('map', 'base_link',  rospy.Time())
        
        #     mavros_pose_msg = PoseStamped()
        #     mavros_pose_msg.header.frame_id = "base_link"
        #     mavros_pose_msg.header.stamp = rospy.Time.now()
        #     mavros_pose_msg.pose.position.x = self.trans.transform.translation.x
        #     mavros_pose_msg.pose.position.y = self.trans.transform.translation.y
        #     mavros_pose_msg.pose.position.z = self.trans.transform.translation.z
        #     mavros_pose_msg.pose.orientation.x = self.trans.transform.rotation.x
        #     mavros_pose_msg.pose.orientation.y = self.trans.transform.rotation.y
        #     mavros_pose_msg.pose.orientation.z = self.trans.transform.rotation.z
        #     mavros_pose_msg.pose.orientation.w = self.trans.transform.rotation.w
            
        #     #publish
        #     self.mavros_pose_pub.publish(mavros_pose_msg)
        #     rospy.loginfo_once("tf published :)")
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #     rospy.logwarn_once("Waiting for tf data from lidar")

        

    def transform_msg(self, header, child, tx, ty, tz, rx, ry, rz, rw): #function populates transformStamped message
        t =  TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = header
        t.child_frame_id = child
        t.transform.translation.x = tx
        t.transform.translation.y = ty
        t.transform.translation.z = tz
        t.transform.rotation.x = rx
        t.transform.rotation.y = ry
        t.transform.rotation.z = rz
        t.transform.rotation.w = rw
        return t

if __name__ == '__main__': #initialise node
    rospy.init_node('Lidar_msg_converter', anonymous=True)
    c = Converter()
    rospy.spin()