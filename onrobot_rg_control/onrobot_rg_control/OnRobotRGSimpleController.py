#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from onrobot_rg_msgs.msg import OnRobotRGOutput

class OnRobotRGSimpleController(Node):
    def __init__(self):
        super().__init__('OnRobotRGSimpleController')
        self.declare_parameter('onrobot/gripper', 'rg2ft')  # Default-Wert ist 'rg2ft'
        self.gtype = self.get_parameter('onrobot/gripper').get_parameter_value().string_value
        self.pub = self.create_publisher(OnRobotRGOutput, 'OnRobotRGOutput', 1)
        self.command = OnRobotRGOutput()
        self.timer = self.create_timer(0.1, self.publisher)
#----------------------------------------------------------
    def genCommand(self, char, command): 
        """Updates the command according to the character entered by the user."""

        if self.gtype == 'rg2ft':
            max_force = 400
            max_width = 1000
        elif self.gtype == 'rg6':
            max_force = 1200
            max_width = 1600
        else:
            rclpy.shutdown()

        if char == 'a':
            command.r_gfr = 200
            command.r_gwd = max_width
            command.r_ctr = 1
            command.out_zero = 1 
        elif char == 'c':
            command.r_gwd= 0
            command.r_ctr = 1
        elif char == 'o':
            command.r_gwd = max_width
            command.r_ctr = 1
        elif char == 'i':
            command.r_gfr += 25
            command.r_gfr = min(max_force, command.r_gfr)
        elif char == 'd':
            command.r_gfr -= 25
            command.r_gfr = max(0, command.r_gfr)
        elif char == 's':
            command.r_ctr = 0
        elif char == 'z':
            command.out_zero = 1
        elif char == 'uz':
            command.out_zero = 0
        elif char != '':
            if char[0] == 'f':
                try:
                    command.r_gfr = max(0, min(max_force, int(char[1:])))
                    command.r_ctr = 1
                except ValueError:
                    pass
            else:
                try:
                    command.r_gwd = min(max_width, int(char))
                    command.r_ctr = 1
                except ValueError:
                    pass

        return command

#----------------------------------------------------------
    def askForCommand(self):
        """Asks the user for a command to send to the gripper."""

        currentCommand = 'Simple OnRobot RG Controller\n-----\nCurrent command:'
        currentCommand += ' r_gfr = ' + str(self.command.r_gfr)
        currentCommand += ', r_gwd = ' + str(self.command.r_gwd)
        currentCommand += ', r_ctr = ' + str(self.command.r_ctr)
        currentCommand += ', out_zero = ' + str(self.command.out_zero)

        print(currentCommand)

        strAskForCommand = '-----\nAvailable commands\n\n'
        strAskForCommand += 'a: Activate\n'
        strAskForCommand += 'c: Close\n'
        strAskForCommand += 'o: Open\n'
        strAskForCommand += 's: STOP current motion\n'
        strAskForCommand += '(0 - max width): Go to that position\n'
        strAskForCommand += 'i: Increase force\n'
        strAskForCommand += 'd: Decrease force\n'
        strAskForCommand += 'z: sets the out_zero bit to 1, so all force and torque values are set to 0\n'
        strAskForCommand += 'uz: sets the out_zero bit to 0, to undo the force and torque bias\n'
        strAskForCommand += 'f0 to f400: Set the force value between 0N and 40N in 1/10N steps(very low values might result in the gripper not moving)\n'

        strAskForCommand += '-->'

        return input(strAskForCommand)
#----------------------------------------------------------
    def publisher(self):
        char = self.askForCommand()  # Entfernen Sie self.command als Argument
        self.command = self.genCommand(char, self.command)
        self.pub.publish(self.command)
#----------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    onrobot_rg_simple_controller = OnRobotRGSimpleController()
    rclpy.spin(onrobot_rg_simple_controller)
    onrobot_rg_simple_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
