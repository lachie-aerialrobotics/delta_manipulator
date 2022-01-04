#!/usr/bin/env python
import rospy
import numpy as np
import message_filters
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

    def callback_ang(self, pos):  #return servo angle message       
        self.theta1, self.theta2, self.theta3, issolved = self.inverse_kinematics(pos.point.x, pos.point.y, pos.point.z)
        
        if issolved == True and pos.point.z <= 0:
            if cache.issolved == False:
                rospy.loginfo("TOOLTIP SETPOINT RETURNED TO WORKSPACE")
            cache.issolved = True
            self.thetb1 = self.rads2bits(self.theta1,cfg.dir1)
            self.thetb2 = self.rads2bits(self.theta2,cfg.dir2)
            self.thetb3 = self.rads2bits(self.theta3,cfg.dir3)
            cache.thetb1 = self.thetb1
            cache.thetb2 = self.thetb2
            cache.thetb3 = self.thetb3    
        else:
            try:
                if cache.issolved == True:
                    rospy.logwarn("TOOLTIP SETPOINT EXCEEDS WORKSPACE")
                cache.issolved = False
                self.thetb1 = cache.thetb1
                self.thetb2 = cache.thetb2
                self.thetb3 = cache.thetb3
            except:
                self.thetb1 = 2048
                self.thetb2 = 2048
                self.thetb3 = 2048

        ang_msg = ServoMsg(self.thetb1,self.thetb2,self.thetb3).msg
        return ang_msg

    def callback_crrnt(self, force): #return servo current message

        I1 = self.torque2current(force.point.x)
        I2 = self.torque2current(force.point.y)
        I3 = self.torque2current(force.point.z)

        crrnt_msg = ServoMsg(I1,I2,I3).msg

        return crrnt_msg

    def inverse_kinematics(self, x, y, z):
        #Solve position kinematics
        E1 = 2 * cfg.L * (y + cfg.a) 
        E2 = -cfg.L * (np.sqrt(3) * (x + cfg.b) + y + cfg.c)
        E3 = cfg.L * (np.sqrt(3) * (x - cfg.b) - y - cfg.c)

        F1 = 2 * z * cfg.L
        F2 = 2 * z * cfg.L
        F3 = 2 * z * cfg.L

        G1 = x**2 + y**2 + z**2 + cfg.a**2 + cfg.L**2 + 2 * y * cfg.a - cfg.l**2
        G2 = x**2 + y**2 + z**2 + cfg.b**2 + cfg.c**2 + cfg.L**2 + 2 * (x * cfg.b + y * cfg.c) - cfg.l**2
        G3 = x**2 + y**2 + z**2 + cfg.b**2 + cfg.c**2 + cfg.L**2 + 2 * (-x * cfg.b + y * cfg.c) - cfg.l**2

        E = np.array([E1,E2,E3])
        F = np.array([F1,F2,F3])
        G = np.array([G1,G2,G3])

        #Quadratic discriminant
       
        issolved = self.solve_checker(E,F,G)
        if issolved == True:
            t = (-F - np.sqrt(E**2 + F**2 - G**2))/(G - E)

            #servo angles in radians
            theta = 2 * np.arctan(t)

            theta1 = theta[0]
            theta2 = theta[1]
            theta3 = theta[2]
            
            issolved = True        
        else:
            issolved = False #no solutions -> stop arm :(
            theta1 = np.nan
            theta2 = np.nan
            theta3 = np.nan

        return theta1, theta2, theta3, issolved

    def solve_checker(self,E,F,G):
        #check that quadratic discriminant is positive
        # (this is the bit that goes wrong when manipulator workspace is exceeded)
        disc = (2 * F)**2 - 4 * (G - E) * (G + E) 
        if (disc[0] >= 0) and (disc[1] >= 0) and (disc[2] >= 0):
            issolved = True
        else:
            issolved = False
        return issolved
    
    def rads2bits(self,theta,dir):
        #convert from radians to bit values recongnised by dynamixels
        if dir == True:
            thetb = int(2048 + 1024 * (theta * (2/np.pi)))
        elif dir == False:
            thetb = int(2048 - 1024 * (theta * (2/np.pi)))
        return thetb 

    def bits2rads(self,thetb,dir):
        #convert from bits back to radians
        if dir ==True:
            theta = float((thetb - 2048)/1024 * np.pi/2)
        elif dir == False:
            theta = float((thetb - 2048)/-1024 * np.pi/2)
        return theta

    def torque2current(self,T):
        #calculate servo current limit to achieve desired torque (taken from datasheet graph)
        I = int((-0.99861 * abs(T) - 0.0286) *1000 / 2.69)
        if I > 2047:
            I = 2047 #do not let current exceed limit
        return I

class cache: #save most recent valid values of servo angles in case inverse kinematics breaks
    issolved = True
    def __init__(self, thetb1, thetb2, thetb3, issolved):
        self.thetb1 = thetb1
        self.thetb2 = thetb2
        self.thetb3 = thetb3
        self.issolved = issolved

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
        self.pub_ang = rospy.Publisher('/servo/setpoint_angles', servo_angles, queue_size=1) # servo angle publisher
        # self.pub_crrnt = rospy.Publisher(robot_name+'/servo/setpoint_currents', servo_angles, queue_size=1) # servo current publisher

        self.sub_pos = message_filters.Subscriber('/tooltip/setpoint_position/local', PointStamped) #target angle subscriber
        # self.sub_force = message_filters.Subscriber(robot_name+'/servo/setpoint_torques', PointStamped) #target force subscriber
    
    def loop(self):
        ts = message_filters.ApproximateTimeSynchronizer([self.sub_pos#, self.sub_force
            ], 1, 100)
        ts.registerCallback(self.tip_callback)
        
    def tip_callback(self, sub_pos#, sub_force
            ): #callback calculates servo angles/torques
        ang = delta().callback_ang(sub_pos)
        self.pub_ang.publish(ang)
        # crrnt = delta().callback_crrnt(sub_force)
        # self.pub_crrnt.publish(crrnt)
        
if __name__ == '__main__': #initialise node and run loop
    rospy.init_node('delta_inverse_kinematics', anonymous=True)
    Controller().loop()
    rospy.spin()