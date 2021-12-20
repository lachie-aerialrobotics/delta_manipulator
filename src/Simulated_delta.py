#! /usr/bin/env python
import rospy
from delta_manipulator.msg import servo_angles

#simple script to simulate pinging angles to dynamixel servos
class Controller:
    def __init__(self): #init params and publishers and subscribers
        robot_name = rospy.get_param('/namespace')
        self.sub_servo_angles_sp = rospy.Subscriber(robot_name+'/servo/setpoint_angles', servo_angles, self.callback, tcp_nodelay=True) 
        self.pub_servo_angles_read = rospy.Publisher(robot_name+'/servo/detected_angles', servo_angles, queue_size=1, tcp_nodelay=True) 

    def callback(self, servo_sp_msg):
        d = rospy.Duration(0.005, 0)
        rospy.sleep(d)
        servo_read_msg = servo_sp_msg
        self.pub_servo_angles_read.publish(servo_read_msg)
        
if __name__ == '__main__': #initialise node
    rospy.init_node('delta_simulator', anonymous=True)
    c = Controller()
    rospy.spin()


