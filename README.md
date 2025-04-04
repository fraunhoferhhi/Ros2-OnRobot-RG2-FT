# onrobot_rg_control Ros2 Foxy

The `onrobot_rg_control` package is designed to control the RG2-FT gripper using ROS2 FOXY and Modbus TCP.
(For Humble change to `main` branch)

## Installation

To use this package, you first need to clone it into your ROS 2 workspace. Use the following command:

```bash
cd ~/ros2_ws/src
```
```
git clone --branch Ros2-Foxy https://github.com/fraunhoferhhi/Ros2-OnRobot-RG2-FT.git
```
# Installing Dependencies
Dependencies and requirements for this package can be installed from the package.xml or requirements.txt files. Execute the following commands to install the necessary dependencies:
# Install dependencies from package.xml
```
rosdep update
```
```
rosdep install --from-paths src --ignore-src -r -y
```
# Install Python dependencies from requirements.txt
for ros2 foxy the pymodbus version should be 2.4.0
```
pip install -r requirements.txt
```
# Build the packages
```
cd ~/ros2_ws/
```
```
colcon build
```
```
source install/setup.bash
```
# Establishing Connection and Starting Control
To establish a connection to the RG2-FT gripper, first start the bringup launch file with the following command:
```
ros2 launch onrobot_rg_control bringup.launch.py
```
In a separate terminal, you can then start the control node to control the gripper:
```
ros2 run onrobot_rg_control onrobot_rg_simple_controller
```
With these steps, you have completed the basic setup and are now ready to control the RG2-FT gripper using ROS and Modbus TCP.
