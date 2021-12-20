#! /usr/bin/env python
import rospy
import numpy as np
import tf2_ros
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, PointStamped, TransformStamped

#script handles broadcasting of tf_frames representing configuration of drone + manipulator

class cfg: #values loaded from ros parameter server
    robot_name = rospy.get_param('/namespace')
    nozzlex = rospy.get_param('/nozzle')
    drone2basex = rospy.get_param('/drone2base_x')  
    drone2basez = rospy.get_param('/drone2base_z')  
    nozzle = np.asarray([0.0, 0.0, -nozzlex])
    base_offset = np.asarray([drone2basex, 0.0, drone2basez])
    base_pitch = rospy.get_param('/base_pitch')
    quat = quaternion_from_euler(np.deg2rad(base_pitch) , 0.0, -np.pi/2)
    

def transform_msg(header, child, tx, ty, tz, rx, ry, rz, rw): #function populates transformStamped message
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


def drone_pose_callback(drone_pose_msg): #broadcast fcu frame
    br_map2fcu = tf2_ros.TransformBroadcaster()
    tf_map2fcu = transform_msg(
        "map", "fcu", 
        drone_pose_msg.pose.position.x, drone_pose_msg.pose.position.y, drone_pose_msg.pose.position.z,
        drone_pose_msg.pose.orientation.x, drone_pose_msg.pose.orientation.y, drone_pose_msg.pose.orientation.z, drone_pose_msg.pose.orientation.w
    )
    br_map2fcu.sendTransform(tf_map2fcu)


def tip_pos_callback(tip_pos_msg): #broadcast manipulator tip frame 
    br_base2platform = tf2_ros.TransformBroadcaster()
    tf_base2platform = transform_msg(
        "base", "platform",
        tip_pos_msg.point.x, tip_pos_msg.point.y, tip_pos_msg.point.z,
        0.0, 0.0, 0.0, 1.0
    )
    br_base2platform.sendTransform(tf_base2platform)


if __name__ == '__main__':
    rospy.init_node('drone_configuration_broadcaster') #initialise node

    #broadcast static frames
    br_static = tf2_ros.StaticTransformBroadcaster()
    tf_fcu2base = transform_msg(
        "fcu", "base",
        cfg.base_offset[0], cfg.base_offset[1], cfg.base_offset[2],
        cfg.quat[0], cfg.quat[1], cfg.quat[2], cfg.quat[3]
    )

    tf_platform2tooltip = transform_msg(
        "platform", "tooltip",
        cfg.nozzle[0], cfg.nozzle[1], cfg.nozzle[2],
        0.0, 0.0, 0.0, 1.0
    )    
    br_static.sendTransform([tf_fcu2base, tf_platform2tooltip])
    
    #subscribe to drone pose and end_effector location
    drone_pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, drone_pose_callback, tcp_nodelay=True)
    tip_pos_sub = rospy.Subscriber(cfg.robot_name+'/tip/detected_position/local', PointStamped, tip_pos_callback)

    rospy.spin()