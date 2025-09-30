# Multiverse Environment Setup

This README provides a complete guide to configure your environment for **Multiverse**, **ROS 2**, and **MuJoCo**, and control the Stretch robot.

---

## 0. Clone the Repository

Clone the repository with all submodules:

```bash
git clone --recursive https://github.com/mitsav01/Multiverse_stretch_robot.git
```

> **Note:** If cloned without `--recursive`:

```bash
cd Multiverse_stretch_robot
git submodule update --init --recursive
```

---

## 1. Source ROS 2 Workspaces

Add the following to your `~/.bashrc` to automatically source ROS 2 workspaces:

```bash
cd /path/to/Multiverse_stretch_robot/Multiverse-ROS-Connector/ros_ws/multiverse_ws2/
colcon build --symlink-install

echo "source /path/to/Multiverse_stretch_robot/Multiverse-ROS-Connector/ros_ws/multiverse_ws2/install/setup.bash" >> ~/.bashrc
```

Reload the shell after editing:

```bash
source ~/.bashrc
```

---

## 2. Define Aliases

Add these aliases to simplify launching the server and simulator:

```bash
alias multiverse_server='/path/to/Multiverse_stretch_robot/Multiverse-ServerClient/bin/multiverse_server'
alias simulate='/path/to/Multiverse_stretch_robot/mujoco-3.3.5/bin/simulate'
```

Usage:

```bash
# Start the Multiverse server
multiverse_server

# In another terminal, launch the simulator
simulate /path/to/mujoco/xml/file
```

---

## 3. Set PYTHONPATH

Include the Multiverse Python client in your `PYTHONPATH`:

```bash
export PYTHONPATH=$PYTHONPATH:/path/to/Multiverse-ClientPy
```

---

## 4. Run an Example

1. Launch the Multiverse server:

```bash
multiverse_server
```

2. Launch the MuJoCo simulation (example scene):

```bash
simulate /path/to/Multiverse_stretch_robot/example/hello_robot_stretch_3/scene_two_cups.xml
```

3. Start ROS 2 controllers and visualize in Rviz2:

```bash
cd /path/to/Multiverse_stretch_robot/example/hello_robot_stretch_3
ros2 launch stretch.launch.py
```

---

## 5. Stretch Robot Teleoperation

This section covers controlling the Stretch robot using two scripts: **Base Rotation** and **Keyboard Teleop**.

### 5.1 Base Rotation Script

**Purpose:** Rotate the base in place at a fixed angular speed.

**Features:**

* Continuous rotation around the Z-axis.
* Uses ROS 2 messages (`TwistStamped`) on the base velocity topic.
* Minimal, single-function node.

**Usage:**

1. Launch the ROS 2 controller:

```bash
ros2 launch stretch.launch.py
```

2. Run the rotation node in a separate terminal:

```bash
./rotate_base.py
```

> The robot will rotate continuously until stopped.

### 5.2 Keyboard Teleop Script

**Purpose:** Full keyboard teleoperation with multi-key support.

**Features:**

* Move forward/backward: `W/S`
* Rotate left/right: `A/D` or `Q/E`
* Stop immediately: `X`
* Increase/decrease speed: `+/-`
* Multi-key buffer for combined motions
* Safe for Mujoco simulation (avoids arrow key conflicts)

**Controls:**

| Action         | Key(s) |
| -------------- | ------ |
| Move forward   | W      |
| Move backward  | S      |
| Rotate left    | A or Q |
| Rotate right   | D or E |
| Stop           | X      |
| Increase speed | +      |
| Decrease speed | -      |

**Usage:**

1. Make the script executable:

```bash
chmod +x stretch_teleop_full.py
```

2. Run the node:

```bash
./stretch_teleop_full.py
```

3. Control the robot using the keyboard keys in simulation or real hardware.
