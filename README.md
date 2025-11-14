# FACTR Interface

A bridge between FACTR and ROS2. Includes force feedback and gravity compensation. The main function is `Factr.act(forces)`, which returns the current positions of the FACTR. 

## Program flow

1. Init

    a. Declare and read ROS parameters

    b. Create publishers and subscribers

    c. Instantiate `Factr`, which abstracts communication with Dynamixels

    d. Instantiate `Gui`, which shows `Factr`'s current state in a Viser web app

    e. Spin controller, which calculates gravity comp and force feedback and sends commands to both `Factr` and the follower (via JointTrajectory messages)

## Quick Start

To launch the FACTR teleoperation node with RViz visualization:

```bash
ros2 launch factr_interface factr_rviz.launch.py
```

This will start:
1. The FACTR teleoperation node
2. Robot State Publisher (publishes TF transforms from URDF)
3. RViz2 with the pre-configured view

## Manual Launch

If you prefer to launch components separately:

### 1. Launch the FACTR teleoperation node:
```bash
ros2 run factr_interface factr_teleop_node
```

### 2. Launch the robot state publisher:
```bash
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:="$(cat install/factr_interface/share/factr_interface/urdf/factr_teleop_franka.urdf)"
```

### 3. Launch RViz2:
```bash
ros2 run rviz2 rviz2 -d install/factr_interface/share/factr_interface/rviz/factr_teleop.rviz
```

## Topics

The following topics are published:

- `/leader_joint_states` (sensor_msgs/JointState): Current joint positions of the leader arm
- `/robot_description` (std_msgs/String): URDF description of the robot
- `/tf` (tf2_msgs/TFMessage): Transform tree for visualization

## Customizing the View

You can save your own RViz configuration by:
1. Arranging the view as desired in RViz
2. File → Save Config As...
3. Save to `src/factr_interface/rviz/factr_teleop.rviz`

## Troubleshooting

### Robot model not showing
- Check that `/robot_description` topic is being published
- Verify that the URDF file exists and is valid

### Transforms not showing
- Ensure the robot_state_publisher is running
- Check that `/leader_joint_states` is being published
- Verify fixed frame is set to `base_link` in RViz

### RViz config file not found
- Make sure you've built the package with `colcon build --symlink-install`
- Check that the rviz config file is in `install/factr_interface/share/factr_interface/rviz/`
