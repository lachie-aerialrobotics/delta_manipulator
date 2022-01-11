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

class Setpoint:
    def __init__(self):   
        #retrieve params from parameter server
        self.rate = rospy.get_param('/rate')
        self.drone2base_x = rospy.get_param('/manipulator/geometry/drone2base_x')
        self.drone2base_z = rospy.get_param('/manipulator/geometry/drone2base_z')
        self.tip_init_x = rospy.get_param('/manipulator/tooltip/tip_init_x')
        self.tip_init_y = rospy.get_param('/manipulator/tooltip/tip_init_y')
        self.tip_init_z = rospy.get_param('/manipulator/tooltip/tip_init_z')
        self.fcu2tip = np.asarray([self.drone2base_x + self.tip_init_x, self.tip_init_y, self.drone2base_z + self.tip_init_z])

        #default values
        self.Aligned = False
        self.yawAligned = False
        self.posAligned = False
        self.index = 0
        self.traj_msg_seq = 0
        self.q = np.asarray([0.0, 0.0, 0.0, 1.0])
        self.p = np.asarray([0.0, 0.0, 0.0])
        self.manip_p = self.fcu2tip
        self.roll_axis = 0.0
        self.pitch_axis = 0.0
        self.yaw_axis = 0.0
        self.throttle_axis = 0.0
        self.fly_trajectory = False
        self.reset_trajectory = False
    

        #Publish target positions for drone and manipulator tooltip
        self.drone_sp_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1, tcp_nodelay=True)
        self.manip_sp_pub = rospy.Publisher('/tooltip/setpoint_position/global', PointStamped, queue_size=1, tcp_nodelay=True)

        #Subscribe to joystick inputs
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback, tcp_nodelay=True)
        self.drone_path_sub = rospy.Subscriber('/path/drone', Path, self.path_callback, tcp_nodelay=True)
        self.manipulator_path_sub = rospy.Subscriber('/path/tooltip', Path, self.manipulator_path_callback, tcp_nodelay=True)

        #init dynamic reconfigure server
        self.srv = Server(JoystickConfig, config_callback)

        #Calculate setpoints
        rospy.Timer(rospy.Duration(1.0/self.rate), self.setpoint_callback, reset=True)

    def path_callback(self, path_msg):
        self.path_msg = path_msg

    def manipulator_path_callback(self, manipulator_path_msg):
        self.manipulator_path_msg = manipulator_path_msg

    def joy_callback(self, joy_msg):
        #assign required axes to variables with sensible names (float between -1.0 and 1.0)
        Yaw_axis = joy_msg.axes[0]
        Throt_axis = joy_msg.axes[1]
        LTrigger_axis = joy_msg.axes[2]
        Roll_axis = joy_msg.axes[3]
        Pitch_axis = joy_msg.axes[4]
        RTrigger_axis = joy_msg.axes[5]
        DPadHoriz_axis = joy_msg.axes[6]
        DPadVert_axis = joy_msg.axes[7]

        #assign required buttons to variables with sensible names (int 0 or 1)
        A_bttn = joy_msg.buttons[0]
        B_bttn = joy_msg.buttons[1]
        X_bttn = joy_msg.buttons[2]
        Y_bttn = joy_msg.buttons[3]
        LShoulder_bttn = joy_msg.buttons[4]
        RShoulder_bttn = joy_msg.buttons[5]
        Back_bttn = joy_msg.buttons[6]
        Start_bttn = joy_msg.buttons[7]
        Logo_bttn = joy_msg.buttons[8]
        LStick_bttn = joy_msg.buttons[9]
        RStick_bttn = joy_msg.buttons[10]

        self.fly_trajectory = toggle(self.fly_trajectory, A_bttn)
        self.reset_trajectory = press(B_bttn)
        self.pitch_axis = Pitch_axis
        self.yaw_axis = Yaw_axis
        self.roll_axis = Roll_axis
        self.throttle_axis = Throt_axis

    def setpoint_callback(self, event):
        if self.reset_trajectory:
            self.reset_traj()
            rospy.loginfo("TRAJECTORY RESET")

        #fly either from trajectory or joystick inputs
        if self.fly_trajectory == True:
            self.check_traj_changed()
            self.move_to_sp()
            self.trajectory_sp_lookup()
        elif self.fly_trajectory == False:
            self.Aligned = False
            self.yawAligned = False
            self.posAligned = False
            self.joystick_sp_generator()
        
        self.check_bounds()

        #assign pose to PoseStamped message
        drone_sp_msg = PoseStamped()
        drone_sp_msg.header.stamp = rospy.Time.now()
        drone_sp_msg.header.frame_id = "map"
        drone_sp_msg.pose.position.x = self.p[0]
        drone_sp_msg.pose.position.y = self.p[1]
        drone_sp_msg.pose.position.z = self.p[2]
        drone_sp_msg.pose.orientation.x = self.q[0]
        drone_sp_msg.pose.orientation.y = self.q[1]
        drone_sp_msg.pose.orientation.z = self.q[2] 
        drone_sp_msg.pose.orientation.w = self.q[3]

        #assign tip position to PointStamped message
        manip_sp_msg = PointStamped()
        manip_sp_msg.header.stamp = rospy.Time.now()
        manip_sp_msg.header.frame_id = "map"
        manip_sp_msg.point.x = self.manip_p[0]
        manip_sp_msg.point.y = self.manip_p[1]
        manip_sp_msg.point.z = self.manip_p[2]

        #publish messages
        self.drone_sp_pub.publish(drone_sp_msg)
        self.manip_sp_pub.publish(manip_sp_msg)


    def joystick_sp_generator(self):
        self.q = q_mult(np.asarray([0.0, 
                                    0.0, 
                                    np.sin(self.yaw_axis * self.yaw_scaling_param), 
                                    np.cos(self.yaw_axis * self.yaw_scaling_param)]), self.q)

        self.p += qv_mult(self.q, np.asarray([self.pitch_axis * self.vel_scaling_param,
                                                self.roll_axis * self.vel_scaling_param,
                                                self.throttle_axis * self.vel_scaling_param]))

        self.manip_p = self.p + qv_mult(self.q, self.fcu2tip)

    def check_bounds(self): #ensure drone setpoint is inside bounding box
        if self.p[0] > self.x_max_pos:
            self.p[0] = self.x_max_pos
        if self.p[0] < self.x_max_neg:
            self.p[0] = self.x_max_neg
        if self.p[1] > self.y_max_pos:
            self.p[1] = self.y_max_pos
        if self.p[1] < self.y_max_neg:
            self.p[1] = self.y_max_neg
        if self.p[2] > self.z_max:
            self.p[2] = self.z_max
        if self.p[2] < 0.0:
            self.p[2] = 0.0

    def check_traj_changed(self):
        if self.path_msg.header.seq != self.traj_msg_seq:
            self.reset_traj()
            self.traj_msg_seq = self.path_msg.header.seq

    def reset_traj(self):
        self.index = 0
        self.Aligned = False
        self.yawAligned = False
        self.posAligned = False
        self.fly_trajectory = False

    def move_to_sp(self):
        if not self.Aligned:
            p_traj_start = np.asarray([self.path_msg.poses[self.index].pose.position.x,
                                        self.path_msg.poses[self.index].pose.position.y,
                                        self.path_msg.poses[self.index].pose.position.z])

            q_traj_start = np.asarray([self.path_msg.poses[self.index].pose.orientation.x,
                                        self.path_msg.poses[self.index].pose.orientation.y,
                                        self.path_msg.poses[self.index].pose.orientation.z,
                                        self.path_msg.poses[self.index].pose.orientation.w])

            drone2traj = p_traj_start - self.p

            drone2traj_norm = drone2traj / np.linalg.norm(drone2traj)

            q_diff = q_mult(self.q, q_conjugate(q_traj_start))

            if not self.yawAligned:
                if np.arcsin(q_diff[2]) > self.yaw_scaling_param:
                    self.q = q_mult(np.asarray([0.0, 
                                                0.0, 
                                                np.sin(-self.yaw_scaling_param), 
                                                np.cos(-self.yaw_scaling_param)]), self.q)   
                elif np.arcsin(q_diff[2]) < -self.yaw_scaling_param:
                    self.q = q_mult(np.asarray([0.0, 
                                                0.0, 
                                                np.sin(self.yaw_scaling_param), 
                                                np.cos(self.yaw_scaling_param)]), self.q)
                else:
                    self.yawAligned = True

            if not self.posAligned:
                if np.linalg.norm(p_traj_start - self.p) > self.vel_scaling_param:
                    self.p += drone2traj_norm * self.vel_scaling_param  
                else:
                    self.posAligned = True

            self.manip_p = self.p + qv_mult(self.q, self.fcu2tip)

            if self.yawAligned and self.posAligned:
                self.Aligned = True

    def trajectory_sp_lookup(self):  
        if self.Aligned:
            if self.index < len(self.path_msg.poses):
                if self.index == 0:
                    rospy.loginfo("Starting trajectory...")

                self.p[0] = self.path_msg.poses[self.index].pose.position.x  
                self.p[1] = self.path_msg.poses[self.index].pose.position.y  
                self.p[2] = self.path_msg.poses[self.index].pose.position.z 

                self.q[0] = self.path_msg.poses[self.index].pose.orientation.x  
                self.q[1] = self.path_msg.poses[self.index].pose.orientation.y  
                self.q[2] = self.path_msg.poses[self.index].pose.orientation.z 
                self.q[3] = self.path_msg.poses[self.index].pose.orientation.w 

                self.manip_p[0] = self.manipulator_path_msg.poses[self.index].pose.position.x 
                self.manip_p[1] = self.manipulator_path_msg.poses[self.index].pose.position.y
                self.manip_p[2] = self.manipulator_path_msg.poses[self.index].pose.position.z
                self.index += 1
            else:
                self.reset_traj()
                rospy.loginfo("Completed trajectory!")

def press(bttn): #register button press
    return bool(bttn)

def toggle(action_state, bttn): #toggle between two states when button is pressed
    if bttn != 0: #i.e. if button is registered as pressed
        action_state = not(action_state) #toggle between two states
    return action_state

def trigger_hold(axis): #change state when trigger axis is held down
    if axis < 0:
        action_state = True
    else:
        action_state = False
    return action_state

def config_callback(config, level): 
    rate = rospy.get_param('/rate')
    Setpoint.vel_scaling_param = config.v_max_fcu/rate
    Setpoint.yaw_scaling_param = config.yaw_max_fcu/rate
    Setpoint.x_max_pos = config.x_max_pos
    Setpoint.x_max_neg = config.x_max_neg
    Setpoint.y_max_pos = config.y_max_pos
    Setpoint.y_max_neg = config.y_max_neg
    Setpoint.z_max = config.z_max
    Setpoint.loop_count = config.repeats
    return config

if __name__ == '__main__': #initialise node
    rospy.init_node('joystick_node', anonymous=True)
    s = Setpoint()
    rospy.spin()