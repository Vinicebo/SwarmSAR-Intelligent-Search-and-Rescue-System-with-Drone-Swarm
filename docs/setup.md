# Setup Guide

## System Requirements

- Ubuntu 22.04 LTS (native or WSL2 on Windows 11)
- 8 GB RAM minimum (16 GB recommended for 30-drone simulation)
- GPU recommended for Gazebo rendering

## Step-by-step Installation

### 1. WSL2 Setup (Windows users only)

```powershell
# In PowerShell (Administrator)
wsl --install -d Ubuntu-22.04
```

Restart, then open Ubuntu 22.04 from the Start menu.

### 2. ROS 2 Humble

Follow the official guide or run:

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

Add to `~/.bashrc`:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. Gazebo + ROS 2 Bridge

```bash
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins
```

### 4. Build the Workspace

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone <repo-url> SwarmSAR

cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 5. Verify Installation

```bash
ros2 launch swarmsar single_drone.launch.py
```

Gazebo should open with a single drone hovering in an empty world.
