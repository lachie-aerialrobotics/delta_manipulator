#! /usr/bin/env python
import rospy
import numpy as np
from quaternion_functions import qv_mult, q_mult, q_conjugate
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from dynamic_reconfigure.server import Server
from delta_manipulator.cfg import TrajectoryConfig

def config_callback(config, level): 
    rospy.loginfo("Calculating new trajectory.....")
    rate = rospy.get_param('/rate')
    drone2base_x = rospy.get_param('/manipulator/geometry/drone2base_x')
    drone2base_z = rospy.get_param('/manipulator/geometry/drone2base_z')
    tip_init_x = rospy.get_param('/manipulator/tooltip/tip_init_x')
    tip_init_y = rospy.get_param('/manipulator/tooltip/tip_init_y')
    tip_init_z = rospy.get_param('/manipulator/tooltip/tip_init_z')
    
    delta_theta = config.v_traj / (config.r_traj * rate)
    a = config.r_traj
    theta = 0.0

    delta_theta_manip = config.v_traj_delta / (config.r_traj_delta * rate)
    a_manip = config.r_traj_delta
    theta_manip = 0.0
    init_manip = np.asarray([drone2base_x + tip_init_x, tip_init_y, drone2base_z + tip_init_z])

    path_msg = Path()
    path_msg.header.frame_id = "map"
    path_msg.header.stamp = rospy.Time.now()
    path_msg.poses = []

    manip_path_msg = Path()
    manip_path_msg.header.frame_id = "map"
    manip_path_msg.header.stamp = rospy.Time.now()
    manip_path_msg.poses = []

    while theta < 2 * np.pi * config.repeats:   
        q = np.asarray([0.0, 0.0, np.sin(config.yaw_traj), np.cos(config.yaw_traj)])
        p = shape(a, theta, config.drone_mode)
        p = p + np.asarray([config.x_traj, config.y_traj, config.h_traj]) #shift from origin

        p_manip = qv_mult(q, shape(a_manip, theta_manip, config.delta_mode) + init_manip) + p
        # p_manip = shape(a_manip, theta_manip, config.delta_mode) + np.asarray([0, 0, config.h_traj_delta])

        pose = PoseStamped()
        pose.header.frame_id = path_msg.header.frame_id
        pose.header.stamp = path_msg.header.stamp
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        pose.pose.position.x = p[0]
        pose.pose.position.y = p[1]
        pose.pose.position.z = p[2]
        path_msg.poses.append(pose)

        manip_pose = PoseStamped()
        manip_pose.header.frame_id = manip_path_msg.header.frame_id
        manip_pose.header.stamp = manip_path_msg.header.stamp
        manip_pose.pose.orientation.x = q[0]
        manip_pose.pose.orientation.y = q[1]
        manip_pose.pose.orientation.z = q[2]
        manip_pose.pose.orientation.w = q[3]
        manip_pose.pose.position.x = p_manip[0]
        manip_pose.pose.position.y = p_manip[1]
        manip_pose.pose.position.z = p_manip[2]
        manip_path_msg.poses.append(manip_pose)

        theta += delta_theta
        theta_manip += delta_theta_manip

    drone_path_pub.publish(path_msg)
    manip_path_pub.publish(manip_path_msg)
    rospy.loginfo("UPDATED DRONE TRAJECTORY")

    return config

def shape(a, t, mode):
    p = np.asarray([0.0, 0.0, 0.0]) #default value of pose quaternion

    if mode == 0: #static position hold
        p[0] = 0.0
        p[1] = 0.0
        p[2] = 0.0
    elif mode == 1: #line in z
        p[0] = 0.0
        p[1] = 0.0
        p[2] = a * np.sin(t)
    elif mode == 2: #line in y
        p[0] = 0.0
        p[1] = a * np.sin(t)
        p[2] = 0.0
    elif mode == 3: #line in x
        p[0] = a * np.sin(t)
        p[1] = 0.0
        p[2] = 0.0
    elif mode == 4: #circle in xy
        p[0] = a * np.sin(t)
        p[1] = a * np.cos(t)
        p[2] = 0.0
    elif mode == 5: #circle in xz
        p[0] = a * np.sin(t)
        p[1] = 0.0
        p[2] = a * np.cos(t)
    elif mode == 6: #figure 8 in xy
        p[0] = a * np.sin(t) * np.cos(t)
        p[1] = a * np.sin(t)
        p[2] = 0.0

    return p

if __name__ == '__main__': #initialise node
    rospy.init_node('trajectory_generator', anonymous=True)
    drone_path_pub = rospy.Publisher('/path/drone', Path, queue_size=1, tcp_nodelay=True)
    manip_path_pub = rospy.Publisher('/path/tooltip', Path, queue_size=1, tcp_nodelay=True)
    srv = Server(TrajectoryConfig, config_callback)
    rospy.spin()