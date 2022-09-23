# iRobot Create3 ROS workspace

A template repo to create a workspace for use with the iRobot Create3 robot. This application uses Python via `rclpy` to control the robot, and is designed to run on a Raspberry Pi 4 running Ubuntu that is connected to the robot.

You can read more on connecting a Raspberry Pi to an iRobot Create 3 in [this blog post](https://jimbobbennett.dev/blogs/irobot-create3-connect-a-pi/).

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