#!/usr/bin/env python
import rospy
import numpy as np
import tf2_ros
from geometry_msgs.msg import PointStamped, TransformStamped
from delta_manipulator.msg import servo_angles

class Controller:
    def __init__(self): #init params and publishers and subscribers
        self.rate = rospy.get_param('/rate')
        self.sp = rospy.get_param('/manipulator/geometry/sp')
        self.sb = rospy.get_param('/manipulator/geometry/sb')
        self.l = rospy.get_param('/manipulator/geometry/l')
        self.L = rospy.get_param('/manipulator/geometry/L')
        self.dir1 = rospy.get_param('/manipulator/servo/dir1')
        self.dir2 = rospy.get_param('/manipulator/servo/dir2')
        self.dir3 = rospy.get_param('/manipulator/servo/dir3')
        self.wb = np.sqrt(3)/6 * self.sb
        self.wp = np.sqrt(3)/6 * self.sp
        self.up = np.sqrt(3)/3 * self.sp
        self.a = self.wb - self.up
        self.b = self.sp / 2 - np.sqrt(3)/2 * self.wb
        self.c = self.wp - 0.5 * self.wb 

        self.servo_sp_msg = servo_angles()

        self.pub_tip_pos = rospy.Publisher('/tooltip/detected_position/local', PointStamped, queue_size=1, tcp_nodelay=True) 

        self.pubBaseJoint1Point = rospy.Publisher('/manipulator/joint/base/1', PointStamped, queue_size=1, tcp_nodelay=True)
        self.pubBaseJoint2Point = rospy.Publisher('/manipulator/joint/base/2', PointStamped, queue_size=1, tcp_nodelay=True)
        self.pubBaseJoint3Point = rospy.Publisher('/manipulator/joint/base/3', PointStamped, queue_size=1, tcp_nodelay=True)
        self.pubElbowJoint1Point = rospy.Publisher('/manipulator/joint/elbow/1', PointStamped, queue_size=1, tcp_nodelay=True)
        self.pubElbowJoint2Point = rospy.Publisher('/manipulator/joint/elbow/2', PointStamped, queue_size=1, tcp_nodelay=True)
        self.pubElbowJoint3Point = rospy.Publisher('/manipulator/joint/elbow/3', PointStamped, queue_size=1, tcp_nodelay=True)
        self.pubPlatJoint1Point = rospy.Publisher('/manipulator/joint/platform/1', PointStamped, queue_size=1, tcp_nodelay=True)
        self.pubPlatJoint2Point = rospy.Publisher('/manipulator/joint/platform/2', PointStamped, queue_size=1, tcp_nodelay=True)
        self.pubPlatJoint3Point = rospy.Publisher('/manipulator/joint/platform/3', PointStamped, queue_size=1, tcp_nodelay=True)

        
        self.sub_servo_angles_det = rospy.Subscriber('/servo/detected_angles', servo_angles, self.detected_callback, tcp_nodelay=True)    
        self.sub_servo_angles_sp = rospy.Subscriber('/servo/setpoint_angles', servo_angles, self.setpoint_callback, tcp_nodelay=True) 
        rospy.Timer(rospy.Duration(1.0/self.rate), self.callback)
        


    def detected_callback(self, servo_sp_msg):
        self.readPositions = rospy.get_param("/manipulator/servo/read_positions")
        if self.readPositions == True:
            self.servo_sp_msg = servo_sp_msg

    def setpoint_callback(self, servo_sp_msg):
        self.readPositions = rospy.get_param("/manipulator/servo/read_positions")
        if self.readPositions == False:
            self.servo_sp_msg = servo_sp_msg

    def callback(self, event):
        thetb1 = self.servo_sp_msg.theta1
        thetb2 = self.servo_sp_msg.theta2
        thetb3 = self.servo_sp_msg.theta3

        solve, x, y, z = self.forwardKinematics(thetb1, thetb2, thetb3)

        if solve == True:
            tip_pos_msg = PointStamped()
            tip_pos_msg.header.frame_id = "manipulator"
            tip_pos_msg.header.stamp = self.servo_sp_msg.header.stamp
            tip_pos_msg.point.x = x
            tip_pos_msg.point.y = y
            tip_pos_msg.point.z = z
            self.pub_tip_pos.publish(tip_pos_msg)

            #broadcast tranform between platform and manipulator
            br_base2platform = tf2_ros.TransformBroadcaster()
            tf_base2platform = TransformStamped()
            tf_base2platform.header.stamp = self.servo_sp_msg.header.stamp
            tf_base2platform.header.frame_id = "manipulator"
            tf_base2platform.child_frame_id = "platform"
            tf_base2platform.transform.translation.x = tip_pos_msg.point.x
            tf_base2platform.transform.translation.y = tip_pos_msg.point.y
            tf_base2platform.transform.translation.z = tip_pos_msg.point.z
            tf_base2platform.transform.rotation.x = 0.0
            tf_base2platform.transform.rotation.y = 0.0
            tf_base2platform.transform.rotation.z = 0.0
            tf_base2platform.transform.rotation.w = 1.0
            br_base2platform.sendTransform(tf_base2platform)

            self.publish_joint_locations(thetb1, thetb2, thetb3, x, y, z)

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

    def publish_joint_locations(self, thetb1, thetb2, thetb3, x, y, z):
        #convert servo angles to radians
        theta1 = self.bits2rads(thetb1, self.dir1)
        theta2 = self.bits2rads(thetb2, self.dir2)
        theta3 = self.bits2rads(thetb3, self.dir3)

        #Base joint locations:
        Bx1 = 0.0
        By1 = -self.wb
        Bz1 = 0.0

        Bx2 = np.sqrt(3)/2 * self.wb
        By2 = 0.5 * self.wb
        Bz2 = 0.0

        Bx3 = -np.sqrt(3)/2 * self.wb
        By3 = 0.5 * self.wb
        Bz3 = 0.0

        baseJoint1PointMsg = PointStamped()
        baseJoint1PointMsg.header.frame_id = "manipulator"
        baseJoint1PointMsg.header.stamp = self.servo_sp_msg.header.stamp
        baseJoint1PointMsg.point.x = Bx1
        baseJoint1PointMsg.point.y = By1
        baseJoint1PointMsg.point.z = Bz1
        self.pubBaseJoint1Point.publish(baseJoint1PointMsg)

        baseJoint2PointMsg = PointStamped()
        baseJoint2PointMsg.header.frame_id = "manipulator"
        baseJoint2PointMsg.header.stamp = self.servo_sp_msg.header.stamp
        baseJoint2PointMsg.point.x = Bx2
        baseJoint2PointMsg.point.y = By2
        baseJoint2PointMsg.point.z = Bz2
        self.pubBaseJoint2Point.publish(baseJoint2PointMsg)

        baseJoint3PointMsg = PointStamped()
        baseJoint3PointMsg.header.frame_id = "manipulator"
        baseJoint3PointMsg.header.stamp = self.servo_sp_msg.header.stamp
        baseJoint3PointMsg.point.x = Bx3
        baseJoint3PointMsg.point.y = By3
        baseJoint3PointMsg.point.z = Bz3
        self.pubBaseJoint3Point.publish(baseJoint3PointMsg)

        #Elbow locations:
        Ex1 = 0
        Ey1 = -self.wb - self.L * np.cos(theta1) + self.up
        Ez1 = -self.L * np.sin(theta1)
            
        Ex2 = np.sqrt(3)/2 * (self.wb + self.L * np.cos(theta2)) - self.sp/2
        Ey2 = 0.5 * (self.wb + self.L * np.cos(theta2)) - self.wp
        Ez2 = - self.L * np.sin(theta2)

        Ex3 = -np.sqrt(3)/2 * (self.wb + self.L * np.cos(theta3)) + self.sp/2
        Ey3 = 0.5 * (self.wb + self.L * np.cos(theta3)) - self.wp
        Ez3 = - self.L * np.sin(theta3)
        
        elbowJoint1PointMsg = PointStamped()
        elbowJoint1PointMsg.header.frame_id = "manipulator"
        elbowJoint1PointMsg.header.stamp = self.servo_sp_msg.header.stamp
        elbowJoint1PointMsg.point.x = Ex1
        elbowJoint1PointMsg.point.y = Ey1
        elbowJoint1PointMsg.point.z = Ez1
        self.pubElbowJoint1Point.publish(elbowJoint1PointMsg)

        elbowJoint2PointMsg = PointStamped()
        elbowJoint2PointMsg.header.frame_id = "manipulator"
        elbowJoint2PointMsg.header.stamp = self.servo_sp_msg.header.stamp
        elbowJoint2PointMsg.point.x = Ex2
        elbowJoint2PointMsg.point.y = Ey2
        elbowJoint2PointMsg.point.z = Ez2
        self.pubElbowJoint2Point.publish(elbowJoint2PointMsg)

        elbowJoint3PointMsg = PointStamped()
        elbowJoint3PointMsg.header.frame_id = "manipulator"
        elbowJoint3PointMsg.header.stamp = self.servo_sp_msg.header.stamp
        elbowJoint3PointMsg.point.x = Ex3
        elbowJoint3PointMsg.point.y = Ey3
        elbowJoint3PointMsg.point.z = Ez3
        self.pubElbowJoint3Point.publish(elbowJoint3PointMsg)

        #Platform link locations:
        Px1 = x
        Py1 = y - self.up
        Pz1 = z

        Px2 = x + self.sp/2
        Py2 = y + self.wp
        Pz2 = z

        Px3 = x -self.sp/2
        Py3 = y + self.wp
        Pz3 = z

        platJoint1PointMsg = PointStamped()
        platJoint1PointMsg.header.frame_id = "manipulator"
        platJoint1PointMsg.header.stamp = self.servo_sp_msg.header.stamp
        platJoint1PointMsg.point.x = Px1
        platJoint1PointMsg.point.y = Py1
        platJoint1PointMsg.point.z = Pz1
        self.pubPlatJoint1Point.publish(platJoint1PointMsg)

        platJoint2PointMsg = PointStamped()
        platJoint2PointMsg.header.frame_id = "manipulator"
        platJoint2PointMsg.header.stamp = self.servo_sp_msg.header.stamp
        platJoint2PointMsg.point.x = Px2
        platJoint2PointMsg.point.y = Py2
        platJoint2PointMsg.point.z = Pz2
        self.pubPlatJoint2Point.publish(platJoint2PointMsg)

        platJoint3PointMsg = PointStamped()
        platJoint3PointMsg.header.frame_id = "manipulator"
        platJoint3PointMsg.header.stamp = self.servo_sp_msg.header.stamp
        platJoint3PointMsg.point.x = Px3
        platJoint3PointMsg.point.y = Py3
        platJoint3PointMsg.point.z = Pz3
        self.pubPlatJoint3Point.publish(platJoint3PointMsg)

if __name__ == '__main__': #initialise node
    rospy.init_node('delta_forward_kinematics', anonymous=True)
    c = Controller()
    rospy.spin()