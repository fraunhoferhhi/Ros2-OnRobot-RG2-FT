#!/usr/bin/env python3
"""
Module comModbusTcp: defines a class which communicates with
OnRobot Grippers using the Modbus/TCP protocol.
"""

import sys
import time
import rclpy
from rclpy.node import Node
import threading
from pymodbus.client.sync import ModbusTcpClient


class Communication(Node):

    def __init__(self, dummy=False):
        super().__init__('communication')
        self.client = None
        self.dummy = dummy
        self.lock = threading.Lock()
#----------------------------------------------------------
    def connectToDevice(self, ip, port):
        """Connects to the client.
           The method takes the IP address and port number
           (as a string, e.g. '192.168.1.1' and '502') as arguments.
        """
        if self.dummy:
            self.get_logger().info(
                self.get_name() +
                ": " +
                sys._getframe().f_code.co_name)
            return

        self.client = ModbusTcpClient(
            ip,
            port=port,
            stopbits=1,
            bytesize=8,
            parity='E',
            baudrate=115200,
            timeout=1)
        self.client.connect()
#----------------------------------------------------------
    def disconnectFromDevice(self):
        """Closes connection."""
        if self.dummy:
            self.get_logger().info(
                self.get_name() +
                ": " +
                sys._getframe().f_code.co_name)
            return

        self.client.close()
#----------------------------------------------------------
    def setProximityOffset(self, ProxOffsets):

        with self.lock:
                self.client.write_register(
                    address=5, value=ProxOffsets[0], unit=65)
                self.client.write_register(
                    address=6, value=ProxOffsets[1], unit=65)
#----------------------------------------------------------
    def sendCommand(self, message):
        """Sends a command to the Gripper.
           The method takes a list of uint8 as an argument.
        """
        if self.dummy:
            self.get_logger().info(
                self.get_name() +
                ": " +
                sys._getframe().f_code.co_name)
            return

        # Send a command to the device (address 0, 2 ~ 4)
        if message != []:
            with self.lock:
                self.client.write_register(
                    address=0, value=message[0], unit=65)
                self.client.write_register(
                    address=2, value=message[1], unit=65)
                self.client.write_register(
                    address=3, value=message[2], unit=65)
                self.client.write_register(
                    address=4, value=message[3], unit=65)
                # self.client.write_register(
                #    address=5, value=message[4], unit=65)
                # self.client.write_register(
                #    address=6, value=message[5], unit=65)
#----------------------------------------------------------
    def getStatus(self):
        """Sends a request to read, wait for the response
           and returns the Gripper status.
           The method gets by specifying register address as an argument.
        """
        response1 = [0] * 2
        response2 = [0] * 26
        response3 = [0] * 1
        if self.dummy:

            self.get_logger().info(
                self.get_name() +
                ": " +
                sys._getframe().f_code.co_name)
            return response1 + response2 + response3

        with self.lock:
            # Get status from the device (address 5 and 6)
            response1 = self.client.read_holding_registers(
                address=5, count=2, unit=65).registers


            # Get status from the device (address 257 ~ 282)
            response2 = self.client.read_holding_registers(
                address=257, count=26, unit=65).registers

            # Get status from the device (address 0)
            response3 = self.client.read_holding_registers(
                address=0, count=1, unit=65).registers

        # Output the result
        return response1 + response2 + response3
#----------------------------------------------------------
def main():
    rclpy.init()
    communication_node = Communication(dummy=False)
    rclpy.spin(communication_node)

    communication_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()