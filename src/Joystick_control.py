#! /usr/bin/env python
import rospy
import numpy as np
from quaternion_functions import qv_mult, q_mult
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
        #default values
        self.q = np.asarray([0.0, 0.0, 0.0, 1.0])
        self.p = np.asarray([0.0, 0.0, 0.0])
        self.t = 0.0
        self.w = 0

        #retrieve params from parameter server
        rate = rospy.get_param('/rate')

        self.drone2base_x = rospy.get_param('/manipulator/geometry/drone2base_x')
        self.drone2base_z = rospy.get_param('/manipulator/geometry/drone2base_z')
        self.tip_init_x = rospy.get_param('/manipulator/tooltip/tip_init_x')
        self.tip_init_y = rospy.get_param('/manipulator/tooltip/tip_init_y')
        self.tip_init_z = rospy.get_param('/manipulator/tooltip/tip_init_z')
        self.fcu2tip = np.asarray([self.drone2base_x + self.tip_init_x, self.tip_init_y, self.drone2base_z + self.tip_init_z])

        #create class instance to store joystick buttons/axes
        self.j = Joystick()

        #Publish target positions for drone and manipulator tooltip
        self.drone_sp_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1, tcp_nodelay=True)
        self.tip_sp_pub = rospy.Publisher('/tooltip/setpoint_position/global', PointStamped, queue_size=1, tcp_nodelay=True)

        #init dynamic reconfigure server
        srv = Server(JoystickConfig, config_callback)

        #Subscribe to joystick inputs
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.j.joy_callback, tcp_nodelay=True)

        #Calculate setpoints
        rospy.Timer(rospy.Duration(1.0/rate), self.setpoint_callback, reset=True)

    def setpoint_callback(self, event):
        if self.j.perform_trajectory:
            self.trajectory_generator()
        else:
            if self.j.control_delta_arm:
                self.delta_joystick_control()
            else:
                self.drone_joystick_control()
            self.tip_p = self.p + qv_mult(self.q, self.fcu2tip) #get tooltip location in global coordinates
        
        self.check_bounds()

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
        tip_msg.point.x = self.tip_p[0]
        tip_msg.point.y = self.tip_p[1]
        tip_msg.point.z = self.tip_p[2]

        #publish messages
        self.drone_sp_pub.publish(pose_msg)
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

    def trajectory_generator(self):
        if self.w <= len(self.path_msg.poses):
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

def config_callback(config, level):
    
    rate = rospy.get_param('/rate')
    drone2base_x = rospy.get_param('/manipulator/geometry/drone2base_x')
    drone2base_z = rospy.get_param('/manipulator/geometry/drone2base_z')
    tip_init_x = rospy.get_param('/manipulator/tooltip/tip_init_x')
    tip_init_y = rospy.get_param('/manipulator/tooltip/tip_init_y')
    tip_init_z = rospy.get_param('/manipulator/tooltip/tip_init_z')
    fcu2tip = np.asarray([drone2base_x + tip_init_x, tip_init_y, drone2base_z + tip_init_z])

    Setpoint.vel_scaling_param = config.v_max_fcu/rate
    Setpoint.yaw_scaling_param = config.yaw_max_fcu/rate
    Setpoint.delta_scaling_param = config.v_max_tooltip/rate
    
    Setpoint.delta_mode = config.delta_mode
    Setpoint.v_traj_delta = config.v_traj_delta
    Setpoint.delta_increment = config.v_traj_delta / (config.r_traj_delta * rate)
    Setpoint.delta_r = config.r_traj_delta

    Setpoint.x_max_pos = config.x_max_pos
    Setpoint.x_max_neg = -config.x_max_neg
    Setpoint.y_max_pos = config.y_max_pos
    Setpoint.y_max_neg = -config.y_max_neg
    Setpoint.z_max = config.z_max

    try:
        update_trajectories = (config.v_traj_drone != Setpoint.v_traj_drone) or \
            (config.r_traj_drone != Setpoint.drone_r) or \
            (config.drone_mode != Setpoint.drone_mode) or \
            (config.v_traj_delta != Setpoint.v_traj_delta) or \
            (config.r_traj_delta != Setpoint.delta_r) or \
            (config.delta_mode != Setpoint.drone_mode)
    except:
        update_trajectories = True
    
    drone_path_pub = rospy.Publisher('/path', Path, queue_size=1, tcp_nodelay=True)
    delta_path_pub = rospy.Publisher('/path/tooltip', Path, queue_size=1, tcp_nodelay=True)

    if update_trajectories == True:
        delta_t = config.v_traj_drone / (config.r_traj_drone * rate)
        a = config.r_traj_drone
        t = 0.0

        tip_delta_t = config.v_traj_delta / (config.r_traj_delta * rate)
        tip_a = config.r_traj_delta
        tip_t = 0.0

        q = np.asarray([0.0, 0.0, 0.0, 1.0])
        p = np.asarray([0.0, 0.0, 0.0])
        tip_p = np.asarray([0.0, 0.0, 0.0])

        Setpoint.path_msg = Path()
        Setpoint.path_msg.header.frame_id = "map"
        Setpoint.path_msg.header.stamp = rospy.Time.now()
        Setpoint.path_msg.poses = []

        Setpoint.delta_path_msg = Path()
        Setpoint.delta_path_msg.header.frame_id = "map"
        Setpoint.delta_path_msg.header.stamp = rospy.Time.now()
        Setpoint.delta_path_msg.poses = []

        while t < 2 * np.pi:   
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()

            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            
            if config.drone_mode == 0: #static position hold
                p[0] = 0.0
                p[1] = 0.0
                p[2] = 0.0
            elif config.drone_mode == 1: #line in z
                p[0] = 0.0
                p[1] = 0.0
                p[2] = a * np.sin(t)
            elif config.drone_mode == 2: #line in y
                p[0] = 0.0
                p[1] = a * np.sin(t)
                p[2] = 0.0
            elif config.drone_mode == 3: #line in x
                p[0] = a * np.sin(t)
                p[1] = 0.0
                p[2] = 0.0
            elif config.drone_mode == 4: #circle in xy
                p[0] = a * np.sin(t)
                p[1] = a * np.cos(t)
                p[2] = 0.0
            elif config.drone_mode == 5: #figure 8 in xy
                p[0] = a * np.sin(t) * np.cos(t)
                p[1] = a * np.sin(t)
                p[2] = 0.0

            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pose.pose.position.z = p[2]

            Setpoint.path_msg.poses.append(pose)

            delta_pose = PoseStamped()
            delta_pose.header.frame_id = "map"
            delta_pose.header.stamp = rospy.Time.now()
            
            delta_pose.pose.orientation = pose.pose.orientation

            if config.delta_mode == 0: #static position hold
                tip_p[0] = 0.0
                tip_p[1] = 0.0
                tip_p[2] = 0.0
            elif config.delta_mode == 1: #line in z
                tip_p[0] = 0.0
                tip_p[1] = 0.0
                tip_p[2] = tip_a * np.sin(tip_t)
            elif config.delta_mode == 2: #line in y
                tip_p[0] = 0.0
                tip_p[1] = tip_a * np.sin(tip_t)
                tip_p[2] = 0.0
            elif config.delta_mode == 3: #line in x
                tip_p[0] = tip_a * np.sin(tip_t)
                tip_p[1] = 0.0
                tip_p[2] = 0.0
            elif config.delta_mode == 4: #circle in xy
                tip_p[0] = tip_a * np.sin(tip_t)
                tip_p[1] = tip_a * np.cos(tip_t)
                tip_p[2] = 0.0
            elif config.delta_mode == 5: #circle in xz
                tip_p[0] = 0.0
                tip_p[1] = tip_a * np.sin(tip_t)
                tip_p[2] = tip_a * np.cos(tip_t)

            tip_p = p + qv_mult(q, tip_p + fcu2tip) #get tooltip location in global coordinates
            
            delta_pose.pose.position.x = tip_p[0]
            delta_pose.pose.position.y = tip_p[1]
            delta_pose.pose.position.z = tip_p[2]

            Setpoint.delta_path_msg.poses.append(delta_pose)

            t += delta_t
            tip_t += tip_delta_t

        drone_path_pub.publish(Setpoint.path_msg)
        delta_path_pub.publish(Setpoint.delta_path_msg)
        rospy.loginfo("UPDATED DRONE TRAJECTORIES")

        Setpoint.drone_mode = config.drone_mode
        Setpoint.v_traj_drone = config.v_traj_drone
        Setpoint.drone_increment = config.v_traj_drone / (config.r_traj_drone * rate)
        Setpoint.drone_r = config.r_traj_drone

    return config

if __name__ == '__main__': #initialise node
    rospy.init_node('joystick_node', anonymous=True)
    s = Setpoint()
    rospy.spin()