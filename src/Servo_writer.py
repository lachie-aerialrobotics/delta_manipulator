#!/usr/bin/env python

################################################################################
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: Ryu Woon Jung (Leon)

import message_filters
import rospy

from delta_manipulator.msg import servo_angles
from dynamixel_sdk import *                    # Uses Dynamixel SDK library

def Initialise():
    # Initialize PortHandler instance
    # Set the port path
    # Get methods and members of PortHandlerLinux or PortHandlerWindows
    portHandler = PortHandler(DEVICENAME)

    # Initialize PacketHandler instance
    # Set the protocol version
    # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    # Initialize GroupSyncWrite instances
    groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION)

    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        quit()

    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        quit()

    # Enable dynamixel torque
    for i in range(3):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[i], ADDR_PRO_TORQUE_ENABLE, True)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % DXL_ID[i])
    
    # # Add parameter storage for present positions
    # for i in range(3):
    #    dxl_addparam_result = groupSyncRead.addParam(DXL_ID[i])
    #    if dxl_addparam_result != True:
    #        print("[ID:%03d] groupSyncRead addparam failed" % DXL_ID[i])
    #        quit()

    return groupSyncWrite, portHandler, packetHandler

def ServoCallback(servo_angle_sub, servo_current_sub): #servo_current_sub):
    # Allocate goal position value into byte array
    dxl_goal_position = [None] * 3
    param_goal_position = [None] * 3
    dxl_goal_position[0] = servo_angle_sub.theta1
    dxl_goal_position[1] = servo_angle_sub.theta2
    dxl_goal_position[2] = servo_angle_sub.theta3
    
    for i in range(3):
        param_goal_position[i] = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[i])), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[i])), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[i])), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[i]))]
        # Add Dynamixel goal position value to the Syncwrite parameter storage
        dxl_addparam_result= groupSyncWrite.addParam(DXL_ID[i], param_goal_position[i])
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL_ID[i])
            #quit()
        
    #SyncWrite goal position    
    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    #Clear Syncwrite parameter storage
    groupSyncWrite.clearParam()

    dxl_current_lim = [None] * 3
    dxl_current_lim[0] = servo_current_sub.theta1
    dxl_current_lim[1] = servo_current_sub.theta2
    dxl_current_lim[2] = servo_current_sub.theta3

    for i in range(3):
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID[i], ADDR_PRO_GOAL_CURRENT, dxl_current_lim[i])
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
    
if __name__ == '__main__':
    # Control table address
    ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
    ADDR_PRO_GOAL_POSITION      = 116
    ADDR_PRO_PRESENT_POSITION   = 132
    ADDR_PRO_GOAL_CURRENT       = 102

    
    # Data Byte Length
    LEN_PRO_GOAL_POSITION       = 4
    LEN_PRO_GOAL_CURRENT        = 2
    LEN_PRO_PRESENT_POSITION    = 4

    # Protocol version
    PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

    # Default setting
    DXL_ID                      = [1,2,3]
    BAUDRATE                    = 3000000             # Dynamixel default baudrate : 57600
    DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller


    groupSyncWrite, portHandler, packetHandler = Initialise()

    rospy.init_node('Servo_writer', anonymous=True)
    robot_name = rospy.get_param('/namespace')
    servo_angle_sub = message_filters.Subscriber(robot_name+'/servo_angles_setpoint', servo_angles) #target angle subscriber
    servo_current_sub = message_filters.Subscriber(robot_name+'/servo_current_lims', servo_angles) #current limit subscriber
    # servo_angle_pub = rospy.Publisher(robot_name+'/servo_angles', servo_angles, queue_size=1) # servo angle publisher
    ts = message_filters.ApproximateTimeSynchronizer([servo_angle_sub, servo_current_sub], 1, 100)
    ts.registerCallback(ServoCallback)
    rospy.spin()

    # Clear Syncread parameter storage
    #groupSyncRead.clearParam()

    #for i in range(3):
    ## Disable Dynamixels Torque
    #    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[i], ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    #    if dxl_comm_result != COMM_SUCCESS:
    #        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    #    elif dxl_error != 0:
    #        print("%s" % packetHandler.getRxPacketError(dxl_error))

    ## Close port
    #portHandler.closePort()