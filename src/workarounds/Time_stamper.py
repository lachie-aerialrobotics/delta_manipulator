#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

def callback(drone_pose): 
    stamped_pose = PoseStamped() 
    stamped_pose = drone_pose
    stamped_pose.header.stamp = rospy.Time.now()
    stamped_pose.header.frame_id = "/tf"  
    pub = rospy.Publisher('/mavros/setpoint_position/local_stamped', PoseStamped, queue_size=1)
    pub.publish(stamped_pose)
       
if __name__ == '__main__': #initialise node and run loop
    rospy.init_node('time_stamp_temp_fix', anonymous=True)
    sub = rospy.Subscriber('/mavros/setpoint_position/local', PoseStamped, callback)
    rospy.spin()