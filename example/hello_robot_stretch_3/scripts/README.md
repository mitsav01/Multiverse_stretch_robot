# Robot Scripts Overview

This repository contains utility and ROS 2 Python scripts for robot simulation, control, and teleoperation. Each script is described in detail below, with links in the table for easy navigation.

## Scripts Table

| Script                                       | Description                                                                                                    |
| -------------------------------------------- | -------------------------------------------------------------------------------------------------------------- |
| [generate_particle.py](#generate_particlepy) | Generates a MuJoCo XML file with free-floating particles in a hollow cup for simulation.                       |
| [gotopose.py](#gotoposepy)                   | ROS 2 node that moves a differential-drive robot to a target pose using odometry and proportional control.     |
| [grasp.py](#grasppy)                         | ROS 2 node that sends joint trajectory commands to a robot gripper to achieve a predefined grasp pose.         |
| [keyboard_teleop.py](#keyboard_teleoppy)     | ROS 2 node for teleoperating a robot base via keyboard input with adjustable speeds.                           |
| [rotate_base.py](#rotate_basepy)             | ROS 2 node that commands a robot base to rotate continuously in place, useful for testing motion or sensors.   |
| [command_jtc.ipynb](#command_jtcipynb)       | Jupyter Notebook for interactive joint trajectory control using sliders and buttons for a manipulator/gripper. |

---

## Detailed Script Descriptions

### generate_particle.py

`generate_particle.py` is a utility script to generate a MuJoCo XML model containing multiple free-floating spherical particles placed inside a hollow cup. This is useful for simulating granular materials, small objects, liquid proxies, or clutter dynamics in a controlled simulation environment.

**Features:**

* Generates a standalone MuJoCo XML file with user-defined number of particles.
* Randomizes particle positions within the cup radius and height, ensuring no collisions with walls.
* Each particle has a `<freejoint>` for full 3D motion, lightweight inertial properties, and random RGBA color.
* Outputs the XML file to the same directory as the script.

---

### gotopose.py

`gotopose.py` is a ROS 2 node that autonomously drives a differential-drive robot to a target position using odometry feedback. It applies a simple proportional controller for linear and angular velocities.

**Features:**

* Subscribes to `/diff_drive_controller/odom` to obtain the robot's current pose.
* Publishes velocity commands to `/cmd_vel` for reaching a target pose.
* Stops automatically when the robot reaches within a specified tolerance of the goal.
* Handles quaternion-to-yaw conversion for angular control.

---

### grasp.py

`grasp.py` is a ROS 2 node that sends joint trajectory commands to a robot manipulator or gripper to reach a predefined grasp configuration.

**Features:**

* Publishes `JointTrajectory` messages to `/joint_trajectory_controller/joint_trajectory`.
* Sends a single trajectory point for moving the manipulator to a grasp-ready pose.
* Joint positions can be adjusted for different grasp scenarios.

---

### keyboard_teleop.py

`keyboard_teleop.py` allows teleoperation of a robot base using keyboard input and ROS 2. It publishes Twist messages for linear and angular motion, with adjustable speed.

**Features:**

* Publishes `TwistStamped` messages to `/cmd_vel`.
* Supports forward/backward movement, rotation, and immediate stop.
* Linear and angular speeds adjustable with `+` and `-` keys.

---

### rotate_base.py

`rotate_base.py` is a ROS 2 node to command a robot base to continuously rotate in place. Useful for testing sensors, scanning the environment, or verifying base motion control.

**Features:**

* Publishes `TwistStamped` messages to `/cmd_vel`.
* Rotates the robot around the vertical Z-axis at a fixed angular speed.
* Timer-based loop with adjustable publishing period for smooth operation.

---

### command_jtc.ipynb

`command_jtc.ipynb` is a Jupyter Notebook that allows interactive control of a robot's joints using `ipywidgets` sliders and buttons. It communicates with ROS 2 using `rclpy` and publishes `JointTrajectory` messages for real-time manipulation.

**Features:**

* Provides sliders for each joint to set target positions interactively.
* `Send` button publishes the selected joint positions as a trajectory.
* `Stop` button safely shuts down the ROS 2 node.
* Runs the node in a background thread, allowing non-blocking interaction.
* Ideal for testing and fine-tuning joint trajectories in simulation or on hardware.

---