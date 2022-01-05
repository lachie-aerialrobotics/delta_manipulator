#! /usr/bin/env python
import rospy
import numpy as np
from quaternion_functions import qv_mult, q_mult, q_conjugate
from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import Joy
from nav_msgs.msg import Path

from dynamic_reconfigure.server import Server
from delta_manipulator.cfg import TrajectoryConfig


def config_callback(config, level): 
    rospy.loginfo("Calculating new trajectory.....")
    rate = rospy.get_param('/rate')
    
    delta_t = config.v_traj / (config.r_traj * rate)
    a = config.r_traj
    t = 0.0

    q = np.asarray([0.0, 0.0, 0.0, 1.0]) #default value of orientation quaternion
    p = np.asarray([0.0, 0.0, 0.0]) #default value of pose quaternion

    path_msg = Path()
    path_msg.header.frame_id = "map"
    path_msg.header.stamp = rospy.Time.now()
    path_msg.poses = []

    while t < 2 * np.pi:   
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()

        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        
        if config.drone_mode == 0: #static position hold
            p[0] = 0.0
            p[1] = 0.0
            p[2] = config.h_traj
        elif config.drone_mode == 1: #line in z
            p[0] = 0.0
            p[1] = 0.0
            p[2] = a * np.sin(t) + config.h_traj
        elif config.drone_mode == 2: #line in y
            p[0] = 0.0
            p[1] = a * np.sin(t)
            p[2] = config.h_traj
        elif config.drone_mode == 3: #line in x
            p[0] = a * np.sin(t)
            p[1] = 0.0
            p[2] = config.h_traj
        elif config.drone_mode == 4: #circle in xy
            p[0] = a * np.sin(t)
            p[1] = a * np.cos(t)
            p[2] = config.h_traj
        elif config.drone_mode == 5: #figure 8 in xy
            p[0] = a * np.sin(t) * np.cos(t)
            p[1] = a * np.sin(t)
            p[2] = config.h_traj

        pose.pose.position.x = p[0]
        pose.pose.position.y = p[1]
        pose.pose.position.z = p[2]

        path_msg.poses.append(pose)

        t += delta_t

    drone_path_pub.publish(path_msg)
    rospy.loginfo("UPDATED DRONE TRAJECTORIES")

    return config


if __name__ == '__main__': #initialise node
    rospy.init_node('trajectory_generator', anonymous=True)
    drone_path_pub = rospy.Publisher('/path', Path, queue_size=1, tcp_nodelay=True)
    srv = Server(TrajectoryConfig, config_callback)
    rospy.spin()