#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from air_hockey_robot.msg import ArmStatus
from air_hockey_robot.msg import MotorCommands

import sys
sys.path.append("~/Documents/air-hockey-robot/src/air_hockey_robot_py/air_hockey_robot_py/") # Set path

# Dynamixel
import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from packet_handler import PacketHandler
from port_handler import PortHandler
from robotis_def import COMM_SUCCESS, DXL_LOBYTE, DXL_HIWORD, DXL_LOWORD, DXL_HIBYTE
from group_sync_read import GroupSyncRead
from group_sync_write import GroupSyncWrite

ADDR_TORQUE_ENABLE          = 64
ADDR_GOAL_POSITION          = 116
ADDR_PROFILE_VELOCITY       = 112
LEN_GOAL_POSITION           = 4         # Data Byte Length
LEN_PROFILE_VELOCITY        = 4         # Data Byte Length
ADDR_PRESENT_POSITION       = 132
LEN_PRESENT_POSITION        = 4         # Data Byte Length
DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
BAUDRATE                    = 57600
PROTOCOL_VERSION            = 2.0
DXL1_ID                     = 1                 # Dynamixel#1 ID : 1
DXL2_ID                     = 2                 # Dynamixel#1 ID : 2
DEVICENAME                  = '/dev/ttyUSB0'
TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
groupSyncWriteP = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
groupSyncWriteV = GroupSyncWrite(portHandler, packetHandler, ADDR_PROFILE_VELOCITY, LEN_PROFILE_VELOCITY)
groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

class MotorCommunication(Node):

    def __init__(self):
        super().__init__('arm_motion_planner')
        self.command_subscriber = self.create_subscription(MotorCommands, 'motor_commands', self.command_callback, 5)
        self.status_publisher = self.create_publisher(ArmStatus, 'arm_status', 5)
        self.get_logger().info('init')
        
    def command_callback(self, msg):
        self.sync_write(msg.goal_position_id1, msg.goal_position_id2, msg.goal_speed_id1, msg.goal_speed_id2)
        # print(msg.goal_position_id1, msg.goal_position_id2, msg.goal_speed_id1, msg.goal_speed_id2)
        p1, p2 = self.sync_read()
        arm_status = ArmStatus()
        arm_status.header.stamp = self.get_clock().now().to_msg()
        arm_status.header.frame_id = msg.header.frame_id
        arm_status.id1.goal_position = msg.goal_position_id1
        arm_status.id2.goal_position = msg.goal_position_id2
        arm_status.id1.present_position = p1
        arm_status.id2.present_position = p2
        arm_status.id1.moving_status = 0
        arm_status.id2.moving_status = 0
        arm_status.current_point.x = 0.0   
        arm_status.current_point.y = 0.0      
        arm_status.current_point.z = 0.0      
        self.status_publisher.publish(arm_status)        

    def sync_write(self, pos1, pos2, vel1, vel2):
        # Allocate goal position value into byte array
        param_goal_velocity1 = [DXL_LOBYTE(DXL_LOWORD(vel1)), DXL_HIBYTE(DXL_LOWORD(vel1)), DXL_LOBYTE(DXL_HIWORD(vel1)), DXL_HIBYTE(DXL_HIWORD(vel1))]
        param_goal_velocity2 = [DXL_LOBYTE(DXL_LOWORD(vel2)), DXL_HIBYTE(DXL_LOWORD(vel2)), DXL_LOBYTE(DXL_HIWORD(vel2)), DXL_HIBYTE(DXL_HIWORD(vel2))]
        # Add Dynamixel#1 goal velocity value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWriteV.addParam(DXL1_ID, param_goal_velocity1)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL1_ID)
            quit()
        # Add Dynamixel#2 goal velocity value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWriteV.addParam(DXL2_ID, param_goal_velocity2)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL2_ID)
            quit()
        # Syncwrite goal velocity
        dxl_comm_result = groupSyncWriteV.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        # Clear syncwrite parameter storage
        groupSyncWriteV.clearParam()
        # Position
        # Allocate goal position value into byte array
        param_goal_position1 = [DXL_LOBYTE(DXL_LOWORD(pos1)), DXL_HIBYTE(DXL_LOWORD(pos1)), DXL_LOBYTE(DXL_HIWORD(pos1)), DXL_HIBYTE(DXL_HIWORD(pos1))]
        param_goal_position2 = [DXL_LOBYTE(DXL_LOWORD(pos2)), DXL_HIBYTE(DXL_LOWORD(pos2)), DXL_LOBYTE(DXL_HIWORD(pos2)), DXL_HIBYTE(DXL_HIWORD(pos2))]
        # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWriteP.addParam(DXL1_ID, param_goal_position1)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL1_ID)
            quit()
        # Add Dynamixel#2 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWriteP.addParam(DXL2_ID, param_goal_position2)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL2_ID)
            quit()
        # Syncwrite goal position
        dxl_comm_result = groupSyncWriteP.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        # Clear syncwrite parameter storage
        groupSyncWriteP.clearParam()

    def sync_read(self):
        # Syncread present position
        dxl_comm_result = groupSyncRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        # Check if groupsyncread data of Dynamixel#1 is available
        dxl_getdata_result = groupSyncRead.isAvailable(DXL1_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" % DXL1_ID)
            quit()
        # Check if groupsyncread data of Dynamixel#2 is available
        dxl_getdata_result = groupSyncRead.isAvailable(DXL2_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" % DXL2_ID)
            quit()
        # Get Dynamixel#1 present position value
        dxl1_present_position = groupSyncRead.getData(DXL1_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        # Get Dynamixel#2 present position value
        dxl2_present_position = groupSyncRead.getData(DXL2_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        return dxl1_present_position, dxl2_present_position

def main(args=None):
    # Dynamixel
    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()

    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()

    # Enable Dynamixel#1 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % DXL1_ID)

    # Enable Dynamixel#2 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % DXL2_ID)

    # Add parameter storage for Dynamixel#1 present position value
    dxl_addparam_result = groupSyncRead.addParam(DXL1_ID)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncRead addparam failed" % DXL1_ID)
        quit()

    # Add parameter storage for Dynamixel#2 present position value
    dxl_addparam_result = groupSyncRead.addParam(DXL2_ID)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncRead addparam failed" % DXL2_ID)
        quit()

    # ROS
    rclpy.init(args=args)
    motor_communication = MotorCommunication()
    rclpy.spin(motor_communication)
    motor_communication.destroy_node()
    rclpy.shutdown()

    # Dynamixel
    groupSyncRead.clearParam()

    # Disable Dynamixel#1 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Disable Dynamixel#2 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Close port
    portHandler.closePort()

if __name__ == '__main__':
    main()
