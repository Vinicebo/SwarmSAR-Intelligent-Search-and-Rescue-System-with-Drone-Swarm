# Setup Guide

## System Requirements

- Ubuntu 26.04 LTS "Resolute Raccoon" (native partition — recommended)
- 8 GB RAM minimum (16 GB recommended for 30-drone simulation)
- GPU recommended for Gazebo rendering

## Stack

| Component | Version |
|-----------|---------|
| ROS 2 | Lyrical Luth (LTS, released May 2026, supported until May 2031) |
| Simulator | Gazebo Jetty (`gz-sim`), the official pair for ROS 2 Lyrical |
| Bridge | `ros_gz` (`ros_gz_sim`, `ros_gz_bridge`, `ros_gz_interfaces`) |

Since ROS 2 Jazzy, Gazebo is distributed as ROS vendor packages through `packages.ros.org` — there is no need for a separate OSRF/Gazebo apt repository anymore.

## Step-by-step Installation

### 1. ROS 2 Lyrical Luth

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y curl gnupg lsb-release

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update
sudo apt install -y ros-lyrical-desktop python3-colcon-common-extensions
```

Add to `~/.bashrc` so you don't need to source manually every session:

```bash
echo "source /opt/ros/lyrical/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Verify the installation:

```bash
ros2 run demo_nodes_cpp talker
# In another terminal:
ros2 run demo_nodes_py listener
```

### 2. Gazebo Jetty + ROS 2 Bridge

```bash
sudo apt install -y ros-lyrical-ros-gz ros-lyrical-ros-gz-sim ros-lyrical-ros-gz-bridge ros-lyrical-ros-gz-interfaces
```

Verify:

```bash
gz sim --version
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
ros2 launch simulator single_drone.launch.py
```

Gazebo should open with a single drone hovering in an empty world.

## Tips

- If Gazebo opens but the drone does not appear, rerun `colcon build` and check for compilation errors.
- For GPU acceleration (Nvidia): ensure Nvidia drivers are active on Ubuntu before launching Gazebo.
- To simulate 30 drones without freezing: lower the physics `real_time_factor` and `max_step_size` in the `.world` file.
- Because ROS 2 Lyrical + Gazebo Jetty is a very recent release combination, exact system plugin names (`gz-sim-*-system`) may shift slightly between point releases. If a plugin fails to load, check the exact error message — it usually names the missing plugin — and cross-reference against [gazebosim.org/docs/latest](https://gazebosim.org/docs/latest/).
