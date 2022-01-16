#! /usr/bin/env python
import rospy
from delta_manipulator.msg import servo_angles

from dynamic_reconfigure.server import Server
from delta_manipulator.cfg import ServoConfig

#simple script to simulate pinging angles to dynamixel servos
class Controller:
    def __init__(self): #init params and publishers and subscribers
        srv = Server(ServoConfig, config_callback)
        self.sub_servo_angles_sp = rospy.Subscriber('/servo/setpoint_angles', servo_angles, self.callback, tcp_nodelay=True) 
        self.pub_servo_angles_read = rospy.Publisher('/servo/detected_angles', servo_angles, queue_size=1, tcp_nodelay=True) 

    def callback(self, servo_sp_msg):
        d = rospy.Duration(0.005)
        rospy.sleep(d)
        servo_read_msg = servo_sp_msg
        self.pub_servo_angles_read.publish(servo_read_msg)

class cfg:
    rate = 100
    readPositions = False
    servo_mode = 0
    set_mode = True

def config_callback(config,level):  
    rospy.set_param("/manipulator/servo/read_positions", config.readPositions)
    if cfg.readPositions != config.readPositions:
        cfg.readPositions = config.readPositions
        if cfg.readPositions == True:
            rospy.loginfo("READING SERVO POSITIONS ENABLED: Servo communication rate capped to 60Hz because reading is hard")
            cfg.rate = 60
        elif cfg.readPositions == False:
            rospy.loginfo("READING SERVO POSITIONS DISABLED: Servo communication rate is default")
            cfg.rate = rospy.get_param('/rate')

    if cfg.servo_mode != config.servo_mode: 
        cfg.servo_mode = config.servo_mode
        cfg.set_mode = True

    return config
        
if __name__ == '__main__': #initialise node
    rospy.init_node('delta_simulator', anonymous=True)
    c = Controller()
    rospy.spin()


