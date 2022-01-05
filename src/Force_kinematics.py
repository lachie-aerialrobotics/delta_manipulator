#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import PointStamped
from delta_manipulator.msg import servo_angles

class cfg:
    sp = rospy.get_param('/manipulator/geometry/sp')
    sb = rospy.get_param('/manipulator/geometry/sb')
    l = rospy.get_param('/manipulator/geometry/l')
    L = rospy.get_param('/manipulator/geometry/L')
    dir1 = rospy.get_param('/manipulator/servo/dir1')
    dir2 = rospy.get_param('/manipulator/servo/dir2')
    dir3 = rospy.get_param('/manipulator/servo/dir3')
    wb = np.sqrt(3)/6 * sb
    wp = np.sqrt(3)/6 * sp
    up = np.sqrt(3)/3 * sp
    a = wb - up
    b = sp / 2 - np.sqrt(3)/2 * wb
    c = wp - 0.5 * wb 

class delta:
    def __init__(self):
        pass

    def callback_crrnt(self, force): #return servo current message

        I1 = self.torque2current(force.theta1)
        I2 = self.torque2current(force.theta2)
        I3 = self.torque2current(force.theta3)

        crrnt_msg = ServoMsg(I1,I2,I3).msg

        return crrnt_msg

    def torque2current(self,T):
        #calculate servo current limit to achieve desired torque (taken from datasheet graph)
        I = int((-0.99861 * abs(T) - 0.0286) *1000 / 2.69)
        if I > 2047:
            I = 2047 #do not let current exceed limit
        return I

class ServoMsg: #class to assign values to servo_angles message format  
    def __init__(self, Theta1, Theta2, Theta3): 
        self.msg = servo_angles()
        self.msg.header.stamp = rospy.Time.now() 
        self.msg.header.frame_id = "servos"
        self.msg.theta1 = Theta1
        self.msg.theta2 = Theta2
        self.msg.theta3 = Theta3  

class Controller: #init publishers and subscribers
    def __init__(self):
        self.pub_crrnt = rospy.Publisher('/servo/current_limits', servo_angles, queue_size=1) # servo current publisher
        self.sub_force = rospy.Subscriber('/servo/torque_limits', servo_angles, self.tip_callback) #target force subscriber
        
    def tip_callback(self, sub_force): #callback calculates servo angles/torques
        crrnt = delta().callback_crrnt(sub_force)
        self.pub_crrnt.publish(crrnt)
        
if __name__ == '__main__': #initialise node and run loop
    rospy.init_node('delta_force_kinematics', anonymous=True)
    c = Controller()
    rospy.spin()