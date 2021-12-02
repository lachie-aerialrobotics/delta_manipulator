#!/usr/bin/env python
import rospy
import numpy as np
import message_filters
from geometry_msgs.msg import PointStamped
from delta_manipulator.msg import servo_angles
from dynamic_reconfigure.server import Server
from delta_manipulator.cfg import DeltaConfig

class geom: #data from dynamic reconfigure
    def __init__(self, sp, sb, l, L, dir1, dir2, dir3):
        self.sp = sp
        self.sb = sb
        self.l = l
        self.L = L
        self.dir1 = dir1
        self.dir2 = dir2
        self.dir3 = dir3

    @staticmethod
    def config_callback(config, level): 
        wb = np.sqrt(3)/6 * config.sb
        wp = np.sqrt(3)/6 * config.sp
        up = np.sqrt(3)/3 * config.sp
        geom.a = wb - up
        geom.b = config.sp / 2 - np.sqrt(3)/2 * wb
        geom.c = wp - 0.5 * wb   
        geom.l = config.l
        geom.L = config.L

        geom.dir1 = config.dir1
        geom.dir2 = config.dir2
        geom.dir3 = config.dir3
        return config

class delta:
    def __init__(self):
        pass

    def callback_ang(self, pos):  #return servo angle message         
        self.theta1, self.theta2, self.theta3, issolved = self.inverse_kinematics(pos.point.x, pos.point.y, pos.point.z)
        
        if issolved == True:
            self.thetb1 = self.rads2bits(self.theta1,geom.dir1)
            self.thetb2 = self.rads2bits(self.theta2,geom.dir2)
            self.thetb3 = self.rads2bits(self.theta3,geom.dir3)
            cache.thetb1 = self.thetb1
            cache.thetb2 = self.thetb2
            cache.thetb3 = self.thetb3
        else:
            try:
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
        E1 = 2 * geom.L * (y + geom.a) 
        E2 = -geom.L * (np.sqrt(3) * (x + geom.b) + y + geom.c)
        E3 = geom.L * (np.sqrt(3) * (x - geom.b) - y - geom.c)

        F1 = 2 * z * geom.L
        F2 = 2 * z * geom.L
        F3 = 2 * z * geom.L

        G1 = x**2 + y**2 + z**2 + geom.a**2 + geom.L**2 + 2 * y * geom.a - geom.l**2
        G2 = x**2 + y**2 + z**2 + geom.b**2 + geom.c**2 + geom.L**2 + 2 * (x * geom.b + y * geom.c) - geom.l**2
        G3 = x**2 + y**2 + z**2 + geom.b**2 + geom.c**2 + geom.L**2 + 2 * (-x * geom.b + y * geom.c) - geom.l**2

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
            rospy.logwarn("Workspace of manipulator exceeded!")
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
    def __init__(self, thetb1, thetb2, thetb3):
        self.thetb1 = thetb1
        self.thetb2 = thetb2
        self.thetb3 = thetb3

class ServoMsg: #class to assign values to servo_angles message format  
    def __init__(self, Theta1, Theta2, Theta3): 
        self.msg = servo_angles()
        self.msg.header.stamp = rospy.Time.now() 
        self.msg.header.frame_id = "/servos"
        self.msg.theta1 = Theta1
        self.msg.theta2 = Theta2
        self.msg.theta3 = Theta3  

class Controller: #init publishers and subscribers
    def __init__(self):
        robot_name = rospy.get_param('/namespace') 
        self.pub_ang = rospy.Publisher(robot_name+'/servo_angles_setpoint', servo_angles, queue_size=1) # servo angle publisher
        self.pub_crrnt = rospy.Publisher(robot_name+'/servo_current_lims', servo_angles, queue_size=1) # servo current publisher

        self.sub_pos = message_filters.Subscriber(robot_name+'/tip_position_local', PointStamped) #target angle subscriber
        self.sub_force = message_filters.Subscriber(robot_name+'/tip_force', PointStamped) #target force subscriber
    
    def loop(self):
        ts = message_filters.ApproximateTimeSynchronizer([self.sub_pos, self.sub_force], 1, 100)
        ts.registerCallback(self.tip_callback)
        
    def tip_callback(self, sub_pos, sub_force): #callback calculates servo angles/torques
        ang = delta().callback_ang(sub_pos)
        self.pub_ang.publish(ang)
        crrnt = delta().callback_crrnt(sub_force)
        self.pub_crrnt.publish(crrnt)
        
if __name__ == '__main__': #initialise node and run loop
    rospy.init_node('delta_main_node', anonymous=True)
    srv = Server(DeltaConfig, geom.config_callback)
    Controller().loop()
    rospy.spin()