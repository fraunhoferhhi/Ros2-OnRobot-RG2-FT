#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from onrobot_rg_msgs.msg import OnRobotRGOutput
from onrobot_rg_msgs.msg import OnRobotRGInput

busy = 0
stop = 0
counter = 0
forces = [0]*6

class TestNode(Node):
    def __init__(self):
        super().__init__('TestNode')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.pub = self.create_publisher(OnRobotRGOutput, 'OnRobotRGOutput', 1)
        self.create_subscription(OnRobotRGInput, 'OnRobotRGInput', self.getSignals, 10)
        self.timer = self.create_timer(0.1, self.mainloop)

    def mainloop(self):
        global stop
        global busy
        global forces
        self.get_logger().info("Start loop")
        command = OnRobotRGOutput()
        if stop:
            stop = 0
            #stopping the movement
            self.get_logger().info("\nSTOP")
            command.rCTR = 0
            self.timer.sleep(1)
        elif not busy:
            command = self.generateCommands(command)
        self.pub.publish(command)

    def getSignals(self, status):
        global busy
        global forces
        busy = status.Busy
        forces = [0]*6
        forces[0]=status.Fx_l
        forces[1]=status.Fy_l
        forces[2]=status.Fz_l

    def generateCommands(command):

        max_force = 400
        max_width = 1000
        global stop
        global busy
        global forces
        global counter

        if counter == 0:
            #activate gripper (open wit max force and set all foces to 0)
            print("\nactivating gripper")
            command.rGFR = max_force
            command.rGWD = max_width
            command.rCTR = 1
            command.outZero = 1
            counter += 1
            #rospy.sleep(0.1)

        elif counter == 1:
            #set a new force value
            print("\nSetting force to 20N")
            command.rGFR = 200
            counter += 1
            #rospy.sleep(0.1)

        elif counter == 2:
            #fully closing gripper
            print("\nfully closing the gripper")
            command.rGWD = 0
            command.rCTR = 1
            #counter += 1
            
            rospy.sleep(0.1)

        elif counter == 3: 
            print("\nshutting down")
            rospy.signal_shutdown("\n")

        return command


def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()