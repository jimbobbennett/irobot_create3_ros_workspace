# iRobot Create3 ROS workspace

A template repo to create a workspace for use with the iRobot Create3 robot. This application uses Python via `rclpy` to control the robot, and is designed to run on a Raspberry Pi 4 running Ubuntu that is connected to the robot.

## Running this workspace

Use the following commands to run this workspace.

Build the package:


```bash
colcon build
```

Source the package environment:

```bash
source ./install/local_setup.sh
```

Run the package:

```bash
ros2 run irobot_create3_example_py control_robot
```