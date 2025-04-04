from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
      return LaunchDescription([
            DeclareLaunchArgument(
                  "ip", default_value="172.24.8.39",
                  description="IP address for the OnRobot gripper"),
            DeclareLaunchArgument(
                  "port", default_value="502",
                  description="Port for the OnRobot gripper"),
            DeclareLaunchArgument(
                  "gripper", default_value="rg2",
                  description="Type of the OnRobot gripper"),
            DeclareLaunchArgument(
                  "dummy", default_value="false",
                  description="Dummy mode for the OnRobot gripper"),
            Node(
                  package="onrobot_rg_control",
                  executable="onrobot_rg_tcp_node",
                  name="OnRobotRGTcpNode",
                  parameters=[
                        {"ip": LaunchConfiguration("ip")},
                        {"port": LaunchConfiguration("port")},
                        {"gripper": LaunchConfiguration("gripper")},
                        {"dummy": LaunchConfiguration("dummy")},
                  ],
                  #output="screen", 
            ),
            # Node(
            #       package="onrobot_rg_modbus_tcp",
            #       executable="comModbusTcp",
            #       name="Communication",
            #       #output="screen", 
            # ),          

            Node(
                  package="onrobot_rg_control",
                  executable="onrobot_rg_status_listener",
                  name="OnRobotRGStatusListener",
                  parameters=[
                        {"ip": LaunchConfiguration("ip")},
                        {"port": LaunchConfiguration("port")},
                        {"gripper": LaunchConfiguration("gripper")},
                        {"dummy": LaunchConfiguration("dummy")},
                  ],
                  #output="screen", 
            ),
      ])