# Setup Guide

## System Requirements

- Ubuntu 22.04 LTS (native partition — recommended)
- 8 GB RAM minimum (16 GB recommended for 30-drone simulation)
- GPU recommended for Gazebo rendering

## Step-by-step Installation

### 1. ROS 2 Humble

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y curl gnupg lsb-release

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu jammy main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions
```

Add to `~/.bashrc` so you don't need to source manually every session:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Verify the installation:

```bash
ros2 run demo_nodes_cpp talker
# In another terminal:
ros2 run demo_nodes_py listener
```

### 2. Gazebo + ROS 2 Bridge

```bash
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins
```

Verify:

```bash
gazebo --version
# Expected: Gazebo multi-robot simulator, version 11.x.x
```

### 3. Python Dependencies

```bash
pip3 install numpy scipy matplotlib
```

### 4. Clone and Build the Workspace

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone <repo-url> SwarmSAR

cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Also add to `~/.bashrc`:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### 5. First Test

```bash
ros2 launch swarmsar single_drone.launch.py
```

Gazebo should open with a single drone hovering in an empty world.

## Tips

- If Gazebo opens but the drone does not appear, rerun `colcon build` and check for compilation errors.
- For GPU acceleration (Nvidia): ensure Nvidia drivers are active on Ubuntu before launching Gazebo.
- To simulate 30 drones without freezing: lower the Gazebo update rate (`<real_time_update_rate>100</real_time_update_rate>` in the `.world` file).
