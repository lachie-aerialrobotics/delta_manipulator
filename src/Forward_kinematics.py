#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import PointStamped
from delta_manipulator.msg import servo_angles

class Controller:
    def __init__(self): #init params and publishers and subscribers
        self.robot_name = rospy.get_param('/namespace')
        self.sp = rospy.get_param('/sp')
        self.sb = rospy.get_param('/sb')
        self.l = rospy.get_param('/l')
        self.L = rospy.get_param('/L')
        self.dir1 = rospy.get_param('/dir1')
        self.dir2 = rospy.get_param('/dir2')
        self.dir3 = rospy.get_param('/dir3')
        self.wb = np.sqrt(3)/6 * self.sb
        self.wp = np.sqrt(3)/6 * self.sp
        self.up = np.sqrt(3)/3 * self.sp
        self.a = self.wb - self.up
        self.b = self.sp / 2 - np.sqrt(3)/2 * self.wb
        self.c = self.wp - 0.5 * self.wb 

        self.sub_servo_angles_sp = rospy.Subscriber(self.robot_name+'/servo/detected_angles', servo_angles, self.callback, tcp_nodelay=True) 
        self.pub_tip_pos = rospy.Publisher(self.robot_name+'/tip/detected_position/local', PointStamped, queue_size=1, tcp_nodelay=True) 

    def callback(self, servo_sp_msg):
        thetb1 = servo_sp_msg.theta1
        thetb2 = servo_sp_msg.theta2
        thetb3 = servo_sp_msg.theta3
        solve, x, y, z = self.forwardKinematics(thetb1, thetb2, thetb3)
        if solve == True:
            tip_pos_msg = PointStamped()
            tip_pos_msg.header.frame_id = "base"
            tip_pos_msg.header.stamp = rospy.Time.now()
            tip_pos_msg.point.x = x
            tip_pos_msg.point.y = y
            tip_pos_msg.point.z = z
            self.pub_tip_pos.publish(tip_pos_msg)
        else:
            rospy.loginfo("Forward Kinematics Solve Failed")

    def forwardKinematics(self, thetb1, thetb2, thetb3): #convert measured servo angles to x/y/z end effector positions
        #convert servo angles to radians
        theta1 = self.bits2rads(thetb1, self.dir1)
        theta2 = self.bits2rads(thetb2, self.dir2)
        theta3 = self.bits2rads(thetb3, self.dir3)
        theta = np.asarray([theta1, theta2, theta3])

        #Calculate end effector x,y,z location based on measured joint angles
        #Elbow locations:
        x1 = 0
        y1 = -self.wb - self.L * np.cos(theta[0]) + self.up
        z1 = -self.L * np.sin(theta[0])
                    
        x2 = np.sqrt(3)/2 * (self.wb + self.L * np.cos(theta[1])) - self.sp/2
        y2 = 0.5 * (self.wb + self.L * np.cos(theta[1])) - self.wp
        z2 = - self.L * np.sin(theta[1])

        x3 = -np.sqrt(3)/2 * (self.wb + self.L * np.cos(theta[2])) + self.sp/2
        y3 = 0.5 * (self.wb + self.L * np.cos(theta[2])) - self.wp
        z3 = - self.L * np.sin(theta[2])

        #Solve intersection of 3 spheres
        r1 = self.l
        r2 = self.l
        r3 = self.l

        if (abs(z3 - z1) < 0.0001) or (abs(z3 - z2) < 0.0001): #Singularity when spheres are in same z plane
            a = 2 * (x3 - x1)
            b = 2 * (y3 - y1)
            c = r1**2 - r3**2 - x1**2 - y1**2 + x3**2 + y3**2
            d = 2 * (x3 - x2)
            e = 2 * (y3 - y2)
            f = r2**2 - r3**2 - x2**2 - y2**2 + x3**2 + y3**2

            x = (c * e - b * f) / (a * e - b * d)
            y = (a * f - c * d) / (a * e - b * d)

            B = -2 * z1
            C = z1**2 - r1**2 + (x - x1)**2 + (y - y1)**2

            if B**2 - 4 * C < 0:
                solve = False
                X = np.asarray([0.0, 0.0, 0.0])
            else:
                solve = True
                z = (-B - np.sqrt(B**2 - 4 * C)) / 2
                X = np.asarray([x, y, z])

        else: #solution for all other z heights
            a11 = 2 * (x3 - x1)
            a12 = 2 * (y3 - y1)
            a13 = 2 * (z3 - z1)

            a21 = 2 * (x3 - x2)
            a22 = 2 * (y3 - y2)
            a23 = 2 * (z3 - z2)

            b1 = r1**2 - r3**2 - x1**2 - y1**2 - z1**2 + x3**2 + y3**2 + z3**2
            b2 = r2**2 - r3**2 - x2**2 - y2**2 - z2**2 + x3**2 + y3**2 + z3**2

            a1 = a11 / a13 - a21 / a23
            a2 = a12 / a13 - a22 / a23
            a3 = b2 / a23 - b1 / a13
            a4 = -a2 / a1
            a5 = -a3 / a1
            a6 = (-a21 * a4 - a22) / a23
            a7 = (b2 - a21 * a5) / a23

            a = a4**2 + 1 + a6**2
            b = 2 * a4 * (a5 - x1) - 2 * y1 + 2 * a6 * (a7 - z1)
            c = a5 * (a5 - 2 * x1) + a7 * (a7 - 2 * z1) + x1**2 + y1**2 + z1**2 - r1**2

            if (b**2 - 4 * a * c) < 0:
                solve = False
                x = np.NaN
                y = np.NaN
                z = np.NaN
            else:
                solve = True

                y1 = (-b - np.sqrt(b**2 - 4 * a * c)) / (2 * a)
                y2 = (-b + np.sqrt(b**2 - 4 * a * c)) / (2 * a)
                x1 = a4 * y1 + a5
                x2 = a4 * y2 + a5
                z1 = a6 * y1 + a7
                z2 = a6 * y2 + a7

                if z1 <= z2:
                    x = x1
                    y = y1
                    z = z1
                elif z2 < z1:
                    x = x2
                    y = y2
                    z = z2

        return solve, x, y, z
                

    def bits2rads(self, thetb, dir):
        #convert from bits back to radians
        if dir ==True:
            theta = float((thetb - 2048)/1024 * np.pi/2)
        elif dir == False:
            theta = float((thetb - 2048)/-1024 * np.pi/2)
        return theta

if __name__ == '__main__': #initialise node
    rospy.init_node('delta_forward_kinematics', anonymous=True)
    c = Controller()
    rospy.spin()