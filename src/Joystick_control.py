#! /usr/bin/env python
import rospy
import numpy as np
from quaternion_functions import qv_mult, q_mult
from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import Joy

from dynamic_reconfigure.server import Server
from delta_manipulator.cfg import JoystickConfig

#script generates trajectory setpoints with joystick inputs
class Joystick:
    #set default values to all buttons
    Yaw_axis = 0.0
    Throt_axis = 0.0
    LTrigger_axis = 1.0
    Roll_axis = 0.0
    Pitch_axis = 0.0
    RTrigger_axis = 1.0
    DPadHoriz_axis = 0.0
    DPadVert_axis = 0.0
    A_bttn = 0
    B_bttn = 0
    X_bttn = 0
    Y_bttn = 0
    LShoulder_bttn = 0
    RShoulder_bttn = 0
    Back_bttn = 0
    Start_bttn = 0
    Logo_bttn = 0
    LStick_bttn = 0
    RStick_bttn = 0

    perform_trajectory = False
    control_delta_arm = False
    reset_delta_arm = False
    reset_trajectory = False
    takeoff = False

    def joy_callback(self, joy_msg):
        #assign required axes to variables with sensible names (float between -1.0 and 1.0)
        self.Yaw_axis = joy_msg.axes[0]
        self.Throt_axis = joy_msg.axes[1]
        self.LTrigger_axis = joy_msg.axes[2]
        self.Roll_axis = joy_msg.axes[3]
        self.Pitch_axis = joy_msg.axes[4]
        self.RTrigger_axis = joy_msg.axes[5]
        self.DPadHoriz_axis = joy_msg.axes[6]
        self.DPadVert_axis = joy_msg.axes[7]

        #assign required buttons to variables with sensible names (int 0 or 1)
        self.A_bttn = joy_msg.buttons[0]
        self.B_bttn = joy_msg.buttons[1]
        self.X_bttn = joy_msg.buttons[2]
        self.Y_bttn = joy_msg.buttons[3]
        self.LShoulder_bttn = joy_msg.buttons[4]
        self.RShoulder_bttn = joy_msg.buttons[5]
        self.Back_bttn = joy_msg.buttons[6]
        self.Start_bttn = joy_msg.buttons[7]
        self.Logo_bttn = joy_msg.buttons[8]
        self.LStick_bttn = joy_msg.buttons[9]
        self.RStick_bttn = joy_msg.buttons[10]

        self.perform_trajectory = self.toggle(self.perform_trajectory, self.A_bttn)
        self.control_delta_arm = self.trigger_hold(self.RTrigger_axis)
        self.reset_delta_arm = self.press(self.Y_bttn)
        self.reset_trajectory = self.press(self.B_bttn)
        self.takeoff = self.press(self.X_bttn)

    def press(self, bttn):
        return bool(bttn)

    def toggle(self, action_state, bttn): #toggle between two states when button is pressed
        if bttn != 0: #i.e. if button is registered as pressed
            action_state = not(action_state) #toggle between two states
        return action_state

    def trigger_hold(self, axis): #change state when trigger axis is held down
        if axis < 0:
            action_state = True
        else:
            action_state = False
        return action_state

class Setpoint:
    def __init__(self):
        #default values
        self.q = np.asarray([0.0, 0.0, 0.0, 1.0])
        self.p = np.asarray([0.0, 0.0, 0.0])

        #retrieve params from parameter server
        robot_name = rospy.get_param('/namespace')
        rate = rospy.get_param('/rate')

        drone2base_x = rospy.get_param('/drone2base_x')
        drone2base_z = rospy.get_param('/drone2base_z')
        tip_init_x = rospy.get_param('/tip_init_x')
        tip_init_y = rospy.get_param('/tip_init_y')
        tip_init_z = rospy.get_param('/tip_init_z')
        self.fcu2tip = np.asarray([drone2base_x + tip_init_x, tip_init_y, drone2base_z + tip_init_z])

        #init dynamic reconfigure server
        srv = Server(JoystickConfig, self.config_callback)

        #create class instance to store joystick buttons/axes
        self.j = Joystick()

        #Publish target positions for drone and manipulator tooltip
        self.drone_sp_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1, tcp_nodelay=True)
        self.tip_sp_pub = rospy.Publisher(robot_name+'/tip/setpoint_position/global', PointStamped, queue_size=1, tcp_nodelay=True)

        #Subscribe to joystick inputs
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.j.joy_callback, tcp_nodelay=True)

        #Calculate setpoints
        rospy.Timer(rospy.Duration(1.0/rate), self.setpoint_callback, reset=True)

    def setpoint_callback(self, event):
        if self.j.perform_trajectory:
            pass
        else:
            if self.j.control_delta_arm:
                self.fcu2tip += np.asarray([self.delta_scaling_param * self.j.Pitch_axis,
                                                self.delta_scaling_param * self.j.Roll_axis,
                                                self.delta_scaling_param * self.j.Throt_axis])
                if self.j.reset_delta_arm:
                    self.fcu2tip = np.asarray([rospy.get_param('/drone2base_x') + rospy.get_param('/tip_init_x'), 
                                                rospy.get_param('/tip_init_y'), 
                                                rospy.get_param('/drone2base_z') + rospy.get_param('/tip_init_z')])
                                        
                    rospy.loginfo("TOOLTIP POSITION RESET")
            else:
                self.q = q_mult(np.asarray([0.0, 
                                                    0.0, 
                                                    np.sin(self.j.Yaw_axis * self.yaw_scaling_param), 
                                                    np.cos(self.j.Yaw_axis * self.yaw_scaling_param)]), self.q)

                self.p += qv_mult(self.q, np.asarray([self.j.Pitch_axis * self.vel_scaling_param,
                                                                    self.j.Roll_axis * self.vel_scaling_param,
                                                                    self.j.Throt_axis * self.vel_scaling_param]))
       
        tip_p = self.p + qv_mult(self.q, self.fcu2tip)

        #assign pose to PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = self.p[0]
        pose_msg.pose.position.y = self.p[1]
        pose_msg.pose.position.z = self.p[2]
        pose_msg.pose.orientation.x = self.q[0]
        pose_msg.pose.orientation.y = self.q[1]
        pose_msg.pose.orientation.z = self.q[2] 
        pose_msg.pose.orientation.w = self.q[3]

        #assign tip position to PointStamped message
        tip_msg = PointStamped()
        tip_msg.header.stamp = rospy.Time.now()
        tip_msg.header.frame_id = "map"
        tip_msg.point.x = tip_p[0]
        tip_msg.point.y = tip_p[1]
        tip_msg.point.z = tip_p[2]

        #publish messages
        self.drone_sp_pub.publish(pose_msg)
        self.tip_sp_pub.publish(tip_msg)

    @staticmethod
    def config_callback(config, level):
        rate = rospy.get_param('/rate')
        Setpoint.vel_scaling_param = config.v_max_fcu/rate
        Setpoint.yaw_scaling_param = config.yaw_max_fcu/rate
        Setpoint.delta_scaling_param = config.v_max_tooltip/rate
        return config

if __name__ == '__main__': #initialise node
    rospy.init_node('joystick_node', anonymous=True)
    s = Setpoint()
    rospy.spin()