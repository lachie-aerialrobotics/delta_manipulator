#! /usr/bin/env python
import rospy
import mavros

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from mavros_msgs.msg import *
from mavros_msgs.srv import *

class Commander():
    mavros_state = State()
    takeoff_height = 1.0
    pose_init = PoseStamped()

    def __init__(self):
        #init subscribers and publishers
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb, tcp_nodelay=True)
        self.drone_path_sub = rospy.Subscriber('/path/drone', Path, self.path_cb, tcp_nodelay=True)
        self.local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, tcp_nodelay=True, queue_size=10)
 
 
        self.takeoff()

        rospy.sleep(10)
        self.landing()

        


        
    def state_cb(self, state):
        self.mavros_state = state

    def takeoff(self):
        rospy.loginfo("WAITING FOR CONNECTION TO FLIGHT CONTROLLER...")
        while not self.mavros_state.connected:
            rate.sleep()
        rospy.loginfo("FLIGHT CONTROLLER CONNECTED!")

        rospy.loginfo("WAITING FOR ARMING...")
        while not self.mavros_state.armed:
            rate.sleep()
        rospy.loginfo("ARMED! -- PLEASE SWITCH TO OFFBOARD")
            
        while not self.mavros_state.mode == "OFFBOARD":
            rate.sleep()
        
        rospy.loginfo("FLYING!")

    def landing(self):
        rospy.loginfo("LANDING!")
        rospy.wait_for_service('/mavros/cmd/land')
        try:
            l = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
            l(altitude = 0)
        except rospy.ServiceException(e):
            print("Service land call failed: %s", e)

if __name__ == '__main__': #initialise node and run loop
    rospy.init_node('mavros_test_node', anonymous=True)
    rate = rospy.Rate(10)
    c = Commander()
    rospy.spin()