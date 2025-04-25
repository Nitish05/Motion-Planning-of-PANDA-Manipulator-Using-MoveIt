# Motion Planning of PANDA Manipulator Using MoveIt (ROS2)

This repository contains two ROS 2 packages that collaboratively simulate and control a Panda robotic manipulator. The packages included are:

- **project4_panda**: Includes robot configurations such as URDF files, MoveIt setups, and predefined gripper positions.
- **package_120385506**: Contains a TypeScript version of the `move_robot` script that executes pick-and-place operations for the Panda manipulator.

---

## Prerequisites

- **Operating System**: Ubuntu 22.04.5 LTS (Jammy Jellyfish)
- **ROS2 Version**: ROS2 Humble Hawksbill ([Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html))

> If encountering issues with ROS2 Humble, consider using ROS2 Galactic with Ubuntu 20.04 instead.

---

## Installation and Setup

### Create a ROS2 Workspace

Create a workspace (if not already created):
```bash
mkdir -p ~/my_ros2_ws/src
cd ~/my_ros2_ws
```

### Clone the Packages

Clone or copy the packages into the `src` directory of your workspace:
```bash
my_ros2_ws/
└── src/
    ├── project4_panda
    └── package_120385506
```

### Build Your Workspace
```bash
colcon build
source install/setup.bash
```

---

## Running the Simulation

### Terminal 1
Launch the robot simulation:
```bash
ros2 launch project4_panda demo.launch.py
```

### Terminal 2
Run the robot control script:
```bash
colcon build
source install/setup.bash
ros2 run package_120385506 move_robot
```

---

## Robot Operation Sequence

Upon executing `move_robot`, the Panda robot performs the following tasks:

1. **Move to Default Pose & Open Gripper**
   - Robot returns to a predefined default position.
   - Ensures the gripper is open.

2. **Pick-up Pose & Grasp**
   - Moves to the object's pickup position.
   - Closes the gripper to grasp the object.

3. **Place Pose & Release**
   - Moves to the designated place location.
   - Opens the gripper, releasing the object.

4. **Return to Default Pose**
   - Robot moves back to the initial default position.

---

## Acknowledgements

Special thanks to the **University of Maryland (UMD)** for providing the original **Project4 Panda** package.
