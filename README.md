# Multiverse Environment Setup

This README explains how to configure your shell environment for working with **Multiverse**, **ROS 2**, and **MuJoCo**.

## 0. Clone the Repository

In the terminal, run the following command to clone the repository (with submodules):

```bash
git clone --recursive https://github.com/mitsav01/Multiverse_stretch_robot.git
```
### Note: If you already cloned the repository without the --recursive flag, run the following to fetch submodules:

```bash
cd Multiverse_stretch_robot
git submodule update --init --recursive
```

## 1. Source ROS 2 Workspaces
Add the following lines to your `~/.bashrc` so the ROS 2 workspaces are sourced automatically:

```bash
cd /path/to/Multiverse_stretch_robot/Multiverse-ROS-Connector/ros_ws/multiverse_ws2/

colcon build --symlink-install

# Source connector workspace
echo "source /path/to/Multiverse_stretch_robot/Multiverse-ROS-Connector/ros_ws/multiverse_ws2/install/setup.bash" >> ~/.bashrc
```
## 2. Aliases

For convenience, define these aliases in your `~/.bashrc`:

```bash
# Start Multiverse server
alias multiverse_server='/path/to/Multiverse_stretch_robot/Multiverse-ServerClient/bin/multiverse_server'

# Run MuJoCo simulate
alias simulate='/path/to/Multiverse_stretch_robot/mujoco-3.3.5/bin/simulate'
```

We can use the above command by simply writimng in a terminal:

```bash
# starts the server
multiverse_server   
```

In another terminal,

```bash
# launches MuJoCo simulator
simulate  /path/to/mujoco/xml/file       
```
## 3. Python Path

To use the Python client library for `Multiverse`, extend your PYTHONPATH:

export PYTHONPATH=$PYTHONPATH:/path/to/Multiverse-ClientPy

## 4. Apply changes

After editing your `~/.bashrc`, reload it with:

```bash
source ~/.bashrc
```

## 5. Run given example

To run an example in a provided `example` directory, please follow the steps below:

1. In a terminal, launch multivese server using given command

```bash
multiverse_server
```

2. In another terminal, launch mujoco simulation from example `scene_two_cups.xml` by:

```bash
simulate /path/to/Multiverse_stretch_robot/example/hello_robot_stretch_3/scene_two_cups.xml
```

3. Launch `ROS2` interface and visualise in `Rviz2` using:

```bash
cd /path/to/Multiverse_stretch_robot/example/hello_robot_stretch_3
ros2 launch controller.launch.py
```
It will also activate controllers from above launch files