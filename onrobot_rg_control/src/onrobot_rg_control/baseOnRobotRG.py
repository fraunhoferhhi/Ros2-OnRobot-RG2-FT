#!/usr/bin/env python3

import rclpy
from onrobot_rg_msgs.msg import OnRobotRGInput

def convertTwoComplement(val):
    """"Convert the uint16 signal to a signed interger"""
    bits=16
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val

class onrobotbaseRG:
    """Base class (communication protocol agnostic) for sending commands
       and receiving the status of the OnRobot RG gripper.
    """

    def __init__(self, gtype):
        # Initiate output message as an empty list
        self.gtype = gtype
        self.message = []

        # Note: after the instantiation,
        # a ".client" member must be added to the object

    def verifyCommand(self, command):
        """Verifies that the value of each variable satisfy its limits."""

        # Verify that each variable is in its correct range
        if self.gtype == 'rg2':
            max_force = 400
            max_width = 1000
        elif self.gtype == 'rg6':
            max_force = 1200
            max_width = 1600
        else:
            rclpy.shutdown(
                self.get_name() +
                ": Select the gripper type from rg2 or rg6.")

        command.r_gfr = max(0, command.r_gfr)
        command.r_gfr = min(max_force, command.r_gfr)
        command.r_gwd = max(0, command.r_gwd)
        command.r_gwd = min(max_width, command.r_gwd)

        # Verify that the selected mode number is available
        if command.rCTR not in [0, 1]:
            rclpy.shutdown(
                self.get_name() +
                ": Select the mode number from" +
                "1 (grip), 0 (stop).")

        # Return the modified command
        return command

    def refreshCommand(self, command):
        """Updates the command which will be sent
           during the next sendCommand() call.
        """

        # Limit the value of each variable
        command = self.verifyCommand(command)

        # Initiate command as an empty list
        self.message = []

        # Build the command with each output variable
        self.message.append(command.out_zero)
        self.message.append(command.r_gfr)
        self.message.append(command.r_gwd)
        self.message.append(command.r_ctr)
        #self.message.append(command.outProxOff_l)
        #self.message.append(command.outProxOff_r)

    def sendCommand(self):
        """Sends the command to the Gripper."""

        self.client.sendCommand(self.message)

    def setProximityOffset(self, ProxOffsets):

        self.client.setProximityOffset(ProxOffsets)

    def getStatus(self):
        """Requests the status from the gripper and
           return it in the OnRobotRGInput msg type.
        """

        # Acquire status from the Gripper
        status = self.client.getStatus()

        # Message to output
        message = OnRobotRGInput()

        # Assign the values to their respective variables
        message.g_fof = 0
        message.g_gwd = status[25]
        message.g_wdf = status[25]

        if status[2] == 0 and status[11] == 0 and status[19] == 0 and status[22] == 0:
            message.g_sta = 0
        else:
            message.g_sta = 1

        message.sta_fing_l = status[2]
        message.sta_fing_r = status[11]
        message.sta_prox_l = status[19]
        message.sta_prox_r = status[22]

        message.prox_off_l = status[0]
        message.prox_off_r = status[1]

        message.fx_l = convertTwoComplement(status[4])
        message.fy_l = convertTwoComplement(status[5])
        message.fz_l = convertTwoComplement(status[6])
        message.tx_l = convertTwoComplement(status[7])
        message.ty_l = convertTwoComplement(status[8])
        message.tz_l = convertTwoComplement(status[9])
        message.fx_r = convertTwoComplement(status[13])
        message.fy_r = convertTwoComplement(status[14])
        message.fz_r = convertTwoComplement(status[15])
        message.tx_r = convertTwoComplement(status[16])
        message.ty_r = convertTwoComplement(status[17])
        message.tz_r = convertTwoComplement(status[18])
        message.prox_l = convertTwoComplement(status[20])
        message.prox_r = convertTwoComplement(status[23])
        message.grip_width = convertTwoComplement(status[25])
        message.busy = status[26]
        message.grip_det = status[27]
        message.in_zero = status[28]

        return message