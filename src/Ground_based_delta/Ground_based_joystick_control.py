#! /usr/bin/env python
import rospy
import numpy as np
from quaternion_functions import qv_mult, q_mult, q_conjugate
from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import Joy
from nav_msgs.msg import Path

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

    def press(self, bttn): #register button press
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
        self.tip_init_x = rospy.get_param('/tooltip/tip_init_x')
        self.tip_init_y = rospy.get_param('/tooltip/tip_init_y')
        self.tip_init_z = rospy.get_param('/tooltip/tip_init_z')

        #default values
        self.q = np.asarray([0.0, 0.0, 0.0, 1.0])
        self.p = np.asarray([self.tip_init_x, self.tip_init_y, self.tip_init_z])
        self.w = 0

        #retrieve params from parameter server
        rate = rospy.get_param('/rate')

        self.moving_on_trajectory = False
        self.done_yawing = False
        self.done_translating = False

        #create class instance to store joystick buttons/axes
        self.j = Joystick()

        #Publish target positions for drone and manipulator tooltip
        self.tip_sp_pub = rospy.Publisher('/tooltip/setpoint_position/global', PointStamped, queue_size=1, tcp_nodelay=True)

        #init dynamic reconfigure server
        srv = Server(JoystickConfig, config_callback)

        #Subscribe to joystick inputs
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.j.joy_callback, tcp_nodelay=True)
        self.path_sub = rospy.Subscriber('/path', Path, self.path_callback, tcp_nodelay=True)

        #Calculate setpoints
        rospy.Timer(rospy.Duration(1.0/rate), self.setpoint_callback, reset=True)

    def setpoint_callback(self, event):
        if self.j.perform_trajectory:
            if self.moving_on_trajectory:
                self.trajectory_follower() 
            else:
                self.move_to_trajectory()
        else:
            self.drone_joystick_control()

        #assign tip position to PointStamped message
        tip_msg = PointStamped()
        tip_msg.header.stamp = rospy.Time.now()
        tip_msg.header.frame_id = "map"
        tip_msg.point.x = self.p[0]
        tip_msg.point.y = self.p[1]
        tip_msg.point.z = self.p[2]

        #publish messages
        self.tip_sp_pub.publish(tip_msg)


    def drone_joystick_control(self):
        self.q = q_mult(np.asarray([0.0, 
                                    0.0, 
                                    np.sin(self.j.Yaw_axis * self.yaw_scaling_param), 
                                    np.cos(self.j.Yaw_axis * self.yaw_scaling_param)]), self.q)

        self.p += qv_mult(self.q, np.asarray([self.j.Pitch_axis * self.vel_scaling_param,
                                                self.j.Roll_axis * self.vel_scaling_param,
                                                self.j.Throt_axis * self.vel_scaling_param]))

    def delta_joystick_control(self):
        self.fcu2tip += np.asarray([self.delta_scaling_param * self.j.Pitch_axis,
                                        self.delta_scaling_param * self.j.Roll_axis,
                                        self.delta_scaling_param * self.j.Throt_axis])

        if self.j.reset_delta_arm:
            if not np.array_equal(self.fcu2tip,np.asarray([self.drone2base_x + self.tip_init_x, self.tip_init_y, self.drone2base_z + self.tip_init_z])):
                self.fcu2tip = np.asarray([self.drone2base_x + self.tip_init_x, self.tip_init_y, self.drone2base_z + self.tip_init_z])                           
                rospy.loginfo("TOOLTIP POSITION RESET")

    def move_to_trajectory(self):
        rate = rospy.get_param('/rate')

        p_traj_start = np.asarray([self.path_msg.poses[0].pose.position.x,
                                    self.path_msg.poses[0].pose.position.y,
                                    self.path_msg.poses[0].pose.position.z])

        drone2traj = p_traj_start - self.p

        drone2traj_norm = drone2traj / np.linalg.norm(drone2traj)

        self.done_yawing = True

        if np.linalg.norm(p_traj_start - self.p) > self.v_traj_drone / rate:
            self.p += drone2traj_norm * self.v_traj_drone / rate  
        else:
            self.done_translating = True

        if self.done_yawing and self.done_translating == True:
            self.moving_on_trajectory = True
            self.done_yawing = False
            self.done_translating = False

    def trajectory_follower(self):  
        if self.w < len(self.path_msg.poses):
            self.p[0] = self.path_msg.poses[self.w].pose.position.x  
            self.p[1] = self.path_msg.poses[self.w].pose.position.y  
            self.p[2] = self.path_msg.poses[self.w].pose.position.z 

            self.q[0] = self.path_msg.poses[self.w].pose.orientation.x  
            self.q[1] = self.path_msg.poses[self.w].pose.orientation.y  
            self.q[2] = self.path_msg.poses[self.w].pose.orientation.z 
            self.q[3] = self.path_msg.poses[self.w].pose.orientation.w 

            self.tip_p[0] = self.delta_path_msg.poses[self.w].pose.position.x 
            self.tip_p[1] = self.delta_path_msg.poses[self.w].pose.position.y
            self.tip_p[2] = self.delta_path_msg.poses[self.w].pose.position.z
            self.w += 1
        else:
            self.w = 0
            self.j.perform_trajectory = False
            self.moving_on_trajectory = False

    def path_callback(self, path_msg):
        self.path_msg = path_msg

def config_callback(config, level): 
    rate = rospy.get_param('/rate')

    Setpoint.vel_scaling_param = config.v_max_fcu/rate
    Setpoint.yaw_scaling_param = config.yaw_max_fcu/rate
    Setpoint.delta_scaling_param = config.v_max_tooltip/rate

    Setpoint.x_max_pos = config.x_max_pos
    Setpoint.x_max_neg = -config.x_max_neg
    Setpoint.y_max_pos = config.y_max_pos
    Setpoint.y_max_neg = -config.y_max_neg
    Setpoint.z_max = config.z_max
    return config

if __name__ == '__main__': #initialise node
    rospy.init_node('joystick_node', anonymous=True)
    s = Setpoint()
    rospy.spin()