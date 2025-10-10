# 🚀 ROS 2 Humble on WSL2 (Ubuntu 22.04) — Ready-to-copy Setup Guide

This is a ready-to-copy, step-by-step guide to set up **ROS 2 Humble** inside **WSL2 (Ubuntu 22.04)** on **Windows 11**. It includes GUI support (RViz/Gazebo via WSLg), developer tools, optional GPU hints, and troubleshooting tips.

> Tested flow: run the PowerShell commands on Windows, then run the Linux commands inside the Ubuntu WSL distro.

---

## Table of contents
1. Prerequisites (Windows)
2. Install WSL2 + Ubuntu 22.04
3. First boot & locale
4. Add ROS 2 apt key and repo
5. Install ROS 2 Humble + dev tools
6. Environment & rosdep
7. Test core demo nodes
8. GUI apps (RViz, Gazebo) using WSLg
9. Optional: GPU / CUDA (WSL GPU passthrough)
10. Useful tips & `.wslconfig`
11. Troubleshooting & verification

---

## 1. Prerequisites (Windows)
- Windows 11 (latest updates recommended).
- Install the latest **NVIDIA** or GPU drivers on Windows if you plan to use GPU features inside WSL.
- PowerShell with administrator privileges for WSL installation.

---

## 2. Install WSL2 (PowerShell as Admin)
```powershell
# Install WSL and Ubuntu 22.04
wsl --install -d Ubuntu-22.04

# If asked, restart your machine
```
After restart, launch the Ubuntu app and create your Linux username/password.

---

## 3. First boot: update & configure locale
Open the Ubuntu 22.04 shell and run:
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # confirm settings
```

---

## 4. Add ROS 2 GPG key (modern method) & repository
```bash
# Install prerequisites
sudo apt install -y software-properties-common curl gnupg lsb-release

# Add ROS 2 key (writes to system keyrings)
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 apt source
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update apt
sudo apt update
```

---

## 5. Install ROS 2 Humble + development tools
```bash
# Install desktop (includes rviz, demos, etc.)
sudo apt install -y ros-humble-desktop ros-dev-tools

# Optional: smaller variant
# sudo apt install -y ros-humble-ros-base
```

---

## 6. Environment setup & rosdep
```bash
# Source ROS in current shell
source /opt/ros/humble/setup.bash

# Make sourcing permanent
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Install python build tools and colcon
sudo apt install -y python3-rosdep python3-colcon-common-extensions python3-argcomplete \
  build-essential git

# Initialize rosdep (only once; requires sudo for init)
sudo rosdep init || true
rosdep update
```

> If `sudo rosdep init` prints an error about `/etc/ros/rosdep/sources.list.d/20-default.list`, check permissions and re-run.

---

## 7. Quick test (demo nodes)
Open two WSL terminals (or two panes in your terminal) and run:
```bash
# Terminal 1: run talker
ros2 run demo_nodes_cpp talker

# Terminal 2: run listener
ros2 run demo_nodes_cpp listener
```
If both run and you see published/received logs, core ROS 2 functionality is working.

---

## 8. GUI apps (RViz, Gazebo) — WSLg
WSLg provides integrated GUI support for many Linux apps.

```bash
# Run RViz
rviz2

# Install Gazebo ROS packages if needed
sudo apt install -y ros-humble-gazebo-ros-pkgs
# Run Gazebo-powered examples (as needed)
```
If RViz or Gazebo fails to open, ensure Windows has WSLg enabled (Windows updates) and restart WSL: `wsl --shutdown` then re-open Ubuntu.

---

## 9. Optional: GPU / CUDA in WSL
If you need GPU acceleration inside WSL (for GPU-accelerated perception or ML):
1. Install **NVIDIA for Windows** WSL driver on Windows (from NVIDIA site).
2. Install the CUDA toolkit inside WSL or use nvidia/cuda containers.
3. Verify inside WSL:
```bash
# requires nvidia drivers on the Windows host
nvidia-smi
```
If `nvidia-smi` shows the GPU, you can use `--gpus` in containers or install CUDA packages.

---

## 10. Optional: Optimize WSL performance — `.wslconfig`
Create `C:\Users\<YourName>\.wslconfig` on Windows (replace `<YourName>`):
```ini
[wsl2]
memory=8GB        # Limits VM memory
processors=4       # Number of CPU cores
swap=8GB
localhostForwarding=true
```
Apply changes with:
```powershell
wsl --shutdown
```
Then restart your Ubuntu distro.

---

## 11. Troubleshooting & verification
- **`rviz2` window not opening:** ensure WSLg is available (Windows 11) and that `DISPLAY` is set by WSL automatically. Restart WSL with `wsl --shutdown`.
- **`ros2` commands not found:** ensure you sourced `/opt/ros/humble/setup.bash` or reopened the shell after adding to `~/.bashrc`.
- **`rosdep` network errors:** check internet connectivity and proxy settings.
- **GUI apps show slow rendering:** try increasing WSL memory/processors in `.wslconfig`.
- **Multiple nodes / ports issues:** exit with `Ctrl+C` and check `ps aux | grep ros` to find leftover processes.

---

## Quick commands reference
```bash
# source ROS
source /opt/ros/humble/setup.bash

# create workspace
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws
colcon build
source install/setup.bash

# run demos
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_cpp listener

# shutdown WSL from PowerShell
wsl --shutdown
```

---

## Notes
- Keep Windows and WSL updated for best compatibility (WSLg, systemd support, GPU features).
- For advanced setups (native Windows GPU + specific CUDA toolkits), follow NVIDIA & Microsoft docs for the latest drivers and compatibility.

---

*Generated: ready-to-copy `WSL2-ROS2-Humble-Setup.md` — paste into your repo and commit.*

