#!/usr/bin/env python
import rospy
import numpy as np
import tf2_ros
from geometry_msgs.msg import PointStamped, PoseStamped

from dynamic_reconfigure.server import Server
from delta_manipulator.cfg import TrajectoryConfig


class Trajectory: 
    t = 0
    def __init__(self):
        robot_name = rospy.get_param('/namespace')
        self.ros_rate = rospy.get_param('/rate')
        rate = rospy.Rate(self.ros_rate) # in Hz

        drone_setpoint_pub  = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)
        tip_pose_pub        = rospy.Publisher(robot_name+'/tip/setpoint_position', PoseStamped, queue_size=1)
        servo_torque_pub    = rospy.Publisher(robot_name+'/servo/torques', PointStamped, queue_size=1)
        drone_pose_pub      = rospy.Publisher('/mavros/local_position/pose', PoseStamped, queue_size=1)
        
        while not rospy.is_shutdown():
            self.drone_callback()
            drone_setpoint_pub.publish(self.drone_pose_msg)
            drone_pose_pub.publish(self.drone_pose_msg)
            self.tip_callback()
            tip_pose_pub.publish(self.tip_pose_msg)
            self.servo_callback()
            servo_torque_pub.publish(self.servo_torque_msg)
            rate.sleep()  
        
    def drone_callback(self):
        self.drone_pose_msg = PoseStamped()
        self.drone_pose_msg.header.frame_id = "map"
        self.drone_pose_msg.header.stamp = rospy.Time.now()

        self.drone_pose_msg.pose.orientation.w = 1.0
        self.drone_pose_msg.pose.orientation.x = 0.0
        self.drone_pose_msg.pose.orientation.y = 0.0
        self.drone_pose_msg.pose.orientation.z = 0.0

        self.t += cfg.v / (cfg.r * self.ros_rate)

        if cfg.mode == 0: #static
            self.drone_pose_msg.pose.position.x = cfg.x
            self.drone_pose_msg.pose.position.y = cfg.y
            self.drone_pose_msg.pose.position.z = cfg.z
        if cfg.mode == 1: #line z
            self.drone_pose_msg.pose.position.y = cfg.x
            self.drone_pose_msg.pose.position.x = cfg.y
            self.drone_pose_msg.pose.position.z = cfg.z + cfg.r * np.cos(self.t)
        if cfg.mode == 2: #line y
            self.drone_pose_msg.pose.position.z = cfg.z
            self.drone_pose_msg.pose.position.x = cfg.x
            self.drone_pose_msg.pose.position.y = cfg.y + cfg.r * np.cos(self.t)
        if cfg.mode == 3: #line x
            self.drone_pose_msg.pose.position.y = cfg.y
            self.drone_pose_msg.pose.position.z = cfg.z
            self.drone_pose_msg.pose.position.x = cfg.x + cfg.r * np.cos(self.t)
        if cfg.mode == 4: #circlexy
            self.drone_pose_msg.pose.position.x = cfg.x + cfg.r * np.cos(self.t)
            self.drone_pose_msg.pose.position.y = cfg.y + cfg.r * np.sin(self.t)
            self.drone_pose_msg.pose.position.z = cfg.z 
        if cfg.mode == 5: #circleyz
            self.drone_pose_msg.pose.position.z = cfg.z + cfg.r * np.cos(self.t)
            self.drone_pose_msg.pose.position.y = cfg.y + cfg.r * np.sin(self.t)
            self.drone_pose_msg.pose.position.x = cfg.x 

    def tip_callback(self):
        self.tip_pose_msg = PoseStamped()
        self.tip_pose_msg.header.frame_id = "map"
        self.tip_pose_msg.header.stamp = rospy.Time.now()

        self.tip_pose_msg.pose.orientation.w = self.drone_pose_msg.pose.orientation.w 
        self.tip_pose_msg.pose.orientation.x = self.drone_pose_msg.pose.orientation.x
        self.tip_pose_msg.pose.orientation.y = self.drone_pose_msg.pose.orientation.y
        self.tip_pose_msg.pose.orientation.z = self.drone_pose_msg.pose.orientation.z 

        self.tip_pose_msg.pose.position.x = self.drone_pose_msg.pose.position.x + 0.8
        self.tip_pose_msg.pose.position.y = self.drone_pose_msg.pose.position.y
        self.tip_pose_msg.pose.position.z = self.drone_pose_msg.pose.position.z

    def servo_callback(self):
        self.servo_torque_msg = PointStamped()
        self.servo_torque_msg.header.frame_id = "servos"
        self.servo_torque_msg.header.stamp = rospy.Time.now()
        self.servo_torque_msg.point.x = cfg.T1
        self.servo_torque_msg.point.y = cfg.T2
        self.servo_torque_msg.point.z = cfg.T3

class cfg:
    def __init__(self, r, x, y, z, v, T1, T2, T3, mode):
        self.r = r
        self.x = x
        self.y = y
        self.z = z
        self.v = v
        self.T1 = T1
        self.T2 = T2
        self.T3 = T3
        self.mode = mode
    @staticmethod
    def config_callback(config, level): 
        cfg.r = config.r
        cfg.x = config.x
        cfg.y = config.y
        cfg.z = config.z
        cfg.v = config.v  
        cfg.T1 = config.T1
        cfg.T2 = config.T2
        cfg.T3 = config.T3
        cfg.mode = config.mode
        return config
         
if __name__ == '__main__':    
    rospy.init_node('talker_target', anonymous=True)
    srv = Server(TrajectoryConfig, cfg.config_callback)
    rospy.sleep(rospy.get_param('/tf_wait')) #sleep gives time for tf frames to initialise and prevents non-fatal (but ugly) time_sync error
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    try:
        Trajectory()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down trajectory publisher node")
        pass