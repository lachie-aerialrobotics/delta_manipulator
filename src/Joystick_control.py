#! /usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import PointStamped, PoseStamped

#script calculates required end-effector position of manipulator to stabilise base perturbations using tf2 frames
class Controller():
    def __init__(self): #init publishers and subscribers
        self.pub_tip_pos = rospy.Publisher(robot_name+'/tip/local_position', PointStamped, queue_size=1) #delta target position publisher
        self.tip_sp_sub = rospy.Subscriber(robot_name+'/tip/setpoint_position', PoseStamped, self.callback) #drone measured pose subscriber

    def callback(self, tip_sp_msg):
        #lookup required transforms
        tf_base2map = tfBuffer.lookup_transform('base','map', rospy.Time.now(), rospy.Duration(rospy.get_param('/tf_buffer_wait')))
        tf_tip2plat = tfBuffer.lookup_transform('tooltip','platform', rospy.Time.now(), rospy.Duration(rospy.get_param('/tf_buffer_wait')))

        #perform successive transforms to get delta-arm target in base coordinates (and including offset for nozzle)
        self.target = tf2_geometry_msgs.do_transform_pose(tip_sp_msg, tf_base2map)
        self.target = tf2_geometry_msgs.do_transform_pose(self.target, tf_tip2plat)  
         
        q_inv * (tf_base2map + tip_sp_msg + q * tf_tip2plat)  

        #publish PointStamped message representing target of delta-manipulator relative to base
        pos_msg = PointStamped()
        pos_msg.header.frame_id = "base"
        pos_msg.header.stamp = rospy.Time.now()
        pos_msg.point.x = self.target.pose.position.x
        pos_msg.point.y = self.target.pose.position.y
        pos_msg.point.z = self.target.pose.position.z
        self.pub_tip_pos.publish(pos_msg) 


if __name__ == '__main__': #initialise node
    rospy.init_node('delta_stabilisation_node', anonymous=True)
    robot_name = rospy.get_param('/namespace')
    c = Controller()
    rospy.spin()


