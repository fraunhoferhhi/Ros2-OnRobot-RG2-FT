#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import onrobot_rg_control.comModbusTcp
import onrobot_rg_control.baseOnRobotRG
from onrobot_rg_msgs.msg import OnRobotRGInput
from onrobot_rg_msgs.msg import OnRobotRGOutput


class OnRobotRGTcpNode(Node):
    def __init__(self):
        super().__init__('OnRobotRGTcpNode')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        # Declare parameters
        self.declare_parameter('gripper','rg2')
        
        self.declare_parameter('dummy', False)
        self.declare_parameter('ip', 'setgripperip')  #Standard-IP-Adresse
        self.declare_parameter('port', 502)  #Standardport

        # Gripper is a RG gripper with a Modbus/TCP connection
        self.gripper = onrobot_rg_control.baseOnRobotRG.onrobotbaseRG(self.get_parameter('gripper').value)
        self.gripper.client = onrobot_rg_control.comModbusTcp.Communication(self.get_parameter('dummy').value)

        # Connects to the ip address received as an argument
        self.gripper.client.connectToDevice(self.get_parameter('ip').value, self.get_parameter('port').value)

        # The Gripper status is published on the topic named 'OnRobotRGInput'
        self.pub = self.create_publisher(OnRobotRGInput, 'OnRobotRGInput', 1)

        # The Gripper command is received from the topic named 'OnRobotRGOutput'
        self.create_subscription(OnRobotRGOutput, 'OnRobotRGOutput', self.gripper.refreshCommand, 10)

        # Set the Proximity Offsets
        ProxOffsets = [230, 170]
        self.gripper.setProximityOffset(ProxOffsets)


        self.prev_msg = []
        self.timer = self.create_timer(0.0005, self.mainLoop)
#----------------------------------------------------------
    def mainLoop(self):
        # Get and publish the Gripper status
        status = self.gripper.getStatus()
        self.pub.publish(status)

        # Send the most recent command
        if not int(format(status.g_sta, '016b')[-1]):  # not busy  
            self.gripper.sendCommand()
        elif not self.prev_msg == self.gripper.message:  # find new message
            self.get_logger().info(": Sending message.")
            self.gripper
        self.prev_msg = self.gripper.message

#----------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)

    node = OnRobotRGTcpNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()