#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from onrobot_rg_msgs.msg import OnRobotRGOutput
from onrobot_rg_msgs.msg import OnRobotRGInput#
import time
from std_msgs.msg import String

class OnRobotCube(Node):
    def __init__(self):
        super().__init__('onrobot_cube_py')
        self.declare_parameter('onrobot/gripper', 'rg2ft')  # Default-Wert ist 'rg2ft'
        self.gtype = self.get_parameter('onrobot/gripper').get_parameter_value().string_value
        self.pub = self.create_publisher(OnRobotRGOutput, 'OnRobotRGOutput', 1)
        self.command = OnRobotRGOutput()
        self.subscription = self.create_subscription(OnRobotRGInput,'OnRobotRGInput',self.listener_callback,10)
        self.timer = self.create_timer(0.1, self.publisher)
        self.position_sub = self.create_subscription(String,"position",self.position_callback, 10)
        self.obj_det_pub = self.create_publisher(String,"obj_det",10)
        self.grip_pub = self.create_publisher(String,"grip_sta",10)
        global obj_det
        global grip_val
        global grip_off
        global grip_det
        global pos_data
        global prox_l
        global prox_r
        global grip_det 
        grip_det = "False"
        grip_off = "False"  
        obj_det = "False"
        grip_val = "gripped"


#----------------------------------------------------------

#----------------------------------------------------------
    def position_callback(self, pos_data):
        global grip_val
        global prox_l
        global prox_r
        global grip_det
        
        if grip_val == "gripped" and prox_l < 400 or prox_r < 400:
            self.grip()
            grip_val = "gripped"
            
            self.publish_grip()

                
        # if pos_data.data == "arrivedp1" and (prox_l < 400 or prox_r < 400):
        #     self.release()
        #     grip_val = "no_obj"
        #     self.publish_grip()
            

#----------------------------------------------------------
    def listener_callback(self, status):
        global obj_det
        global grip_val
        global prox_l
        global prox_r
        global grip_det

        pub_counter = 0
        self.publisher()
        prox_l = status.prox_l
        prox_r = status.prox_r
        grip_det = status.g_wdf 

        #print("status.sta_prox_l: ", status.prox_l)
        # while grip_val == "False" and (status.prox_l < 400 or status.prox_r < 400):
        #     self.grip()
        #     self.publisher()    
        #     time.sleep(2)
        #     grip_val = "True"
        #     if grip_val == "True":
        #         self.release()
        #         self.publisher()
        #         time.sleep(2)
        #         grip_val = "False"

        #         break
#----------------------------------------------------------
    def grip(self):
        global grip_val
        self.command.r_ctr = 1
        self.command.r_gfr = 200
        self.command.r_gwd = 250  
        return self.command
#----------------------------------------------------------
    def release(self):
        self.command.r_ctr = 1
        self.command.r_gfr = 200
        self.command.r_gwd = 600
        return self.command 
            
#----------------------------------------------------------
    def publisher(self):
        self.pub.publish(self.command)
#----------------------------------------------------------
    def obj_pub(self, obj_det):
        msg = String()
        msg.data = obj_det
        self.obj_det_pub.publish(msg)
#----------------------------------------------------------
    def publish_grip(self):
        global grip_val
        msg = String()
        msg.data = grip_val
        self.grip_pub.publish(msg)
#----------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    onrobot_rg_simple_controller = OnRobotCube()
    rclpy.spin(onrobot_rg_simple_controller)
    onrobot_rg_simple_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
