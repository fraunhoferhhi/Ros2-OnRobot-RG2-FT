#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from onrobot_rg_msgs.msg import OnRobotRGOutput
from onrobot_rg_msgs.msg import OnRobotRGInput

class TestNode(Node):
    def __init__(self):
        super().__init__('TestNode')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.pub = self.create_publisher(OnRobotRGOutput, 'OnRobotRGOutput', 1)
        self.create_subscription(OnRobotRGInput, 'OnRobotRGInput', self.getSignals, 10)
        self.timer = self.create_timer(0.1, self.mainloop)

        self.busy = 0
        self.stop = 0
        self.counter = 0
        self.forces = [0]*6

    def mainloop(self):
        self.get_logger().info("Start loop")
        command = OnRobotRGOutput()
        if self.stop:
            self.stop = 0
            self.get_logger().info("\nSTOP")
            command.r_ctr = 0
            self.get_clock().sleep(1)
        elif not self.busy:
            command = self.generateCommands(command)
        self.pub.publish(command)

    def getSignals(self, status):
        self.busy = status.busy
        self.forces = [0]*6
        self.forces[0]=status.fx_l
        self.forces[1]=status.fy_l
        self.forces[2]=status.fz_l
        self.forces[3]=status.fx_r
        self.forces[4]=status.fy_r
        self.forces[5]=status.fz_r

    def generateCommands(self, command):
        max_force = 400
        max_width = 1000
        global stop
        global busy
        global forces
        global counter

        if self.counter == 0:
            #activate gripper (open wit max force and set all foces to 0)
            print("\nactivating gripper")
            command.r_gfr = max_force
            command.r_gwd = max_width
            command.r_ctr = 1
            command.out_zero = 1
            self.counter += 1
            self.pub.publish(command)
            time.sleep(0.1)

        elif self.counter == 1:
            #set a new force value
            print("\nSetting force to 20N")
            command.r_gfr = 200
            self.counter += 1
            self.pub.publish(command)
            time.sleep(0.1)

        elif self.counter == 2:
            #move to a new position
            print("\nMoving to new position")
            command.r_gwd = 300
            command.r_ctr = 1
            self.pub.publish(command)
            self.counter += 1

        elif self.counter == 3:
            #fully open gripper again
            print("\nfully opening the gripper again")
            command.r_gwd = max_width
            command.r_ctr = 1
            self.pub.publish(command)
            self.counter += 1

        elif self.counter == 4:
            #set a new force value
            print("\nSetting force to 20N")
            command.r_gfr = 50
            self.pub.publish(command)
            self.counter += 1
            time.sleep(2)

        elif self.counter == 5:
            #fully closing gripper
            print("\nfully closing the gripper")
            command.r_gwd = 0
            command.r_ctr = 1
            self.counter += 1
            stop = 1
            self.pub.publish(command)
            time.sleep(1)


        elif self.counter == 6:
            #stopping the movement
            time.sleep(2)
            #continuing the movement
            print("\ncontinue the movement")
            command.r_ctr = 1
            self.pub.publish(command)
            self.counter += 1

        elif self.counter == 7:
            #printing some sensor readings
            output = '\nHere are all force values:'
            output += ' fx_l = ' + str(self.forces[0])
            output += ' fy_l = ' + str(self.forces[1]) 
            output += ' fz_l = ' + str(self.forces[2]) 
            output += ' fx_r = ' + str(self.forces[3]) 
            output += ' fy_r = ' + str(self.forces[4]) 
            output += ' fz_r = ' + str(self.forces[5])
            print(output)
            self.pub.publish(command)
            self.counter += 1

        elif self.counter == 8: 
            print("\nshutting down")
            rclpy.shutdown()

        return command



def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()