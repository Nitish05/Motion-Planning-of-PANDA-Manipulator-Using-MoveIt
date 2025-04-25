# Motion-Planning-of-PANDA-Manipulator-Using-MoveIt

This repository contains two ROS 2 packages that work together to simulate and control a Panda robot. The packages are:

- **project4_panda** – Contains the robot configurations (URDF, MoveIt configurations, etc.) and named gripper positions.
- **package_120169595** – Contains the `move_robot` script that plans and executes a pick-and-place operation for the Panda robot.

---

## Prerequisites

- Make sure have the **Ubuntu-22.04.5 (Jammy Jellyfish)** LTS installed 
- This Project used **ROS2 Humble Hawksbill** so make sure to install that too.
   Link for ROS2 Humble Documentation - https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
- If ROS2 Humble is not working, Try to use ROS2 Galactic and Ubuntu 20.04.

---

## Installation and Build

### Create or locate your workspace

If you don’t have a workspace yet, create one:

```bash
mkdir -p ~/my_ros2_ws/src
cd ~/my_ros2_ws
```

## Copy or clone the packages into the `src` folder of your workspace:
```bash
my_ros2_ws/
└── src/
    ├── project4_panda
    └── package_120169595
```

## Build Your Workspace:
```bash
colcon build
source install/setup.bash
```
## To run the simulation, run these lines in your workspace:

## Terminal 1:
```bash
ros2 launch project4_panda demo.launch.py
```

## Terminal 2:
```bash
colcon build
source install/setup.bash
ros2 run package_120169595 move_robot
```

## Robot Operation
When you run move_robot, the following sequence occurs:

- **Move to default position & open gripper:**

     The robot moves to a default position and ensures the gripper is open.

- **Move to pickup pose & close gripper:**

     The robot moves to the pickup pose and closes the gripper, simulating grasping an object.

- **Move to place pose & open gripper:**

     The robot moves to the place pose and opens the gripper, simulating placing the object.

- **Return to default position:**

     Finally, the robot moves back to the default position.

## Acknowledgements

I would like to acknowledge the **University of Maryland (UMD)** for providing me the original **Project4 Panda** package.




