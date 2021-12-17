#! /usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import Joy

# from dynamic_reconfigure.server import Server
# from delta_manipulator.cfg import JoystickConfig

#script generates trajectory setpoints with joystick inputs

# class cfg:
#     def __init__(self, v_max):
#         self.v_max = v_max
#     @staticmethod
#     def config_callback(config, level): 
#         cfg.v_max = config.v_max
class button_state:
    flight_mode_switch = 0
    delta_reset_switch = 0
    delta_control_mode = 0
class Controller:
    def __init__(self): #init publishers and subscribers
        self.scaling_param = 0.01
        self.delta_scaling_param = 0.001
        self.landing_speed = 0.1

        self.yaw_quat = np.asarray([1.0, 0.0, 0.0, 0.0])
        self.p_vec = np.asarray([0.0, 0.0, 0.0])

        self.yaw_cmd = 0.0
        self.throt_cmd = 0.0
        self.roll_cmd = 0.0
        self.pitch_cmd = 0.0

        self.flight_mode = 0
        self.delta_control_mode = 0
        self.reset_delta = 0

        robot_name = rospy.get_param('/namespace')
        rate = rospy.get_param('/rate')
        drone2base_x = rospy.get_param('/drone2base_x')
        drone2base_z = rospy.get_param('/drone2base_z')
        tip_init_x = rospy.get_param('/tip_init_x')
        tip_init_y = rospy.get_param('/tip_init_y')
        tip_init_z = rospy.get_param('/tip_init_z')

        self.fcu2tip = np.asarray([drone2base_x + tip_init_x, tip_init_y, drone2base_z + tip_init_z])
        self.fcu2tip_init = self.fcu2tip

        # srv = Server(JoystickConfig, cfg.config_callback)
        self.drone_pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1, tcp_nodelay=True)
        self.tip_sp_pub = rospy.Publisher(robot_name+'/tip/setpoint_position/global', PointStamped, queue_size=1, tcp_nodelay=True)
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        rospy.Timer(rospy.Duration(1.0/rate), self.pose_callback)

    def joy_callback(self, joy_msg):
        self.yaw_cmd = joy_msg.axes[0]
        self.throt_cmd = joy_msg.axes[1]
        self.roll_cmd = joy_msg.axes[3]
        self.pitch_cmd = joy_msg.axes[4]

        flight_mode_switch = joy_msg.buttons[0]

        if flight_mode_switch != button_state.flight_mode_switch:
            self.flight_mode = int(not(self.flight_mode))
            if self.flight_mode == 0:
                rospy.loginfo(["JOYSTICK FLIGHT MODE"])
            elif self.flight_mode == 1:
                rospy.loginfo(["TRAJECTORY FLIGHT MODE"])

        delta_control_switch = joy_msg.axes[5]

        if delta_control_switch < 0:
            self.delta_control_mode = 1
            if self.delta_control_mode != button_state.delta_control_mode:
                button_state.delta_control_mode = 1
                rospy.loginfo(["DELTA CONTROL ENGAGED"])
        elif delta_control_switch >= 0:
            self.delta_control_mode = 0
            if self.delta_control_mode != button_state.delta_control_mode:
                button_state.delta_control_mode = 0
                rospy.loginfo(["DELTA CONTROL DISENGAGED"])
                  
        delta_reset_switch = joy_msg.buttons[5]
        if delta_reset_switch != button_state.delta_reset_switch:
            if delta_reset_switch == 1:
                self.reset_delta = 1
                rospy.loginfo(["DELTA ORIGIN RESET"])

        function_B = joy_msg.buttons[1]
        function_X = joy_msg.buttons[2]
        function_Y = joy_msg.buttons[3]

    def pose_callback(self, event):
        p = PoseStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = "map"


        if self.flight_mode == 0:
            # calculate setpoint from joystick inputs
            if self.delta_control_mode == 0:
                self.yaw_quat = q_mult(np.asarray([np.cos(self.yaw_cmd * self.scaling_param), 0.0, 0.0, np.sin(self.yaw_cmd * self.scaling_param)]), self.yaw_quat)

                self.p_vec += qv_mult(self.yaw_quat, np.asarray([self.pitch_cmd * self.scaling_param,
                                    self.roll_cmd * self.scaling_param,
                                    self.throt_cmd * self.scaling_param]))

            elif self.delta_control_mode == 1:
                self.fcu2tip += np.asarray([self.delta_scaling_param * self.pitch_cmd,
                                                self.delta_scaling_param * self.roll_cmd,
                                                self.delta_scaling_param * self.throt_cmd])
            if self.reset_delta == 1:
                self.fcu2tip = self.fcu2tip_init
                self.reset_delta = 0
            
        elif self.flight_mode == 1:
            pass
        
        p.pose.position.x = self.p_vec[0]
        p.pose.position.y = self.p_vec[1]
        p.pose.position.z = self.p_vec[2]
        p.pose.orientation.w = self.yaw_quat[0]
        p.pose.orientation.x = self.yaw_quat[1]
        p.pose.orientation.y = self.yaw_quat[2]
        p.pose.orientation.z = self.yaw_quat[3] 
        
        tip = PointStamped()
        tip.header.stamp = rospy.Time.now()
        tip.header.frame_id = "map"

        tip_pos_vec = self.p_vec + qv_mult(self.yaw_quat, self.fcu2tip)

        tip.point.x = tip_pos_vec[0]
        tip.point.y = tip_pos_vec[1]
        tip.point.z = tip_pos_vec[2]

        self.drone_pose_pub.publish(p)
        self.tip_sp_pub.publish(tip)

def qv_mult(q1, v1):
    q2 = np.insert(v1, 0, 0.0)
    return q_mult(q_mult(q1, q2), q_conjugate(q1))[1:]

def q_mult(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return w, x, y, z

def q_conjugate(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


if __name__ == '__main__': #initialise node
    rospy.init_node('joystick_node', anonymous=True)
    c = Controller()
    rospy.spin()