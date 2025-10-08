# 🐢 Learning-ROS2

A personal workspace for learning and experimenting with **ROS 2 Humble** using Docker, GPU acceleration (NVIDIA RTX 4060), and VS Code integration. This README documents setup, installation, and usage so anyone can reproduce the environment.

---

## 📖 What is ROS 2?

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It provides:

- A publish/subscribe communication system between nodes
- Tools for simulation (Gazebo, RViz2)
- Libraries for robotics, AI, and machine learning integration
- Support for C++ and Python development

---

## 📚 Table of contents

- [Overview](#-overview)
- [Design choices](#-design-choices)
- [Prerequisites (Host: Manjaro Linux)](#-prerequisites-host-manjaro-linux)
- [Recommended (Docker) installation](#-recommended-docker-installation)

  - [NVIDIA / GPU passthrough](#nvidia--gpu-passthrough)
  - [Docker Compose example](#docker-compose-example)
  - [Common container workflow](#common-container-workflow)

- [Developing inside the container (VS Code)](#developing-inside-the-container-vs-code)
- [Building & running ROS 2 packages](#building--running-ros-2-packages)
- [Alternative installation methods](#-alternative-installation-methods)
- [Tips & troubleshooting](#tips--troubleshooting)
- [Resources & further reading](#resources--further-reading)
- [License](#license)

---

## 🏁 Overview

This repository prioritizes a **Docker-first** approach so you get a reproducible, clean development environment with minimal interference from host packages. That also makes GPU acceleration, VS Code integration, and CI-friendly builds much simpler.

**Why Docker?**

- Isolates ROS 2 Humble environment from host package manager (Manjaro)
- Easier to enable NVIDIA GPU support via `--gpus` and `nvidia-container-toolkit`
- Quick reproducible setup for teammates or future you

---

## ⚙️ Design choices

1. **Docker (Recommended)** – reproducible, GPU-enabled, minimal host changes.
2. **AUR packages** – quicker to get binaries but can break with rolling releases.
3. **Source build** – most flexible but time-consuming; only recommended for maintainers or for experimental changes to core packages.

---

## 🧾 Prerequisites (Host: Manjaro Linux)

These steps assume you have an NVIDIA GPU (RTX 4060) and want GPU acceleration inside the container.

1. Install NVIDIA drivers (host):

```bash
# verify drivers are active
nvidia-smi
```

2. Install Docker and enable the service:

```bash
sudo pacman -Syu docker
sudo systemctl enable --now docker
```

3. Install NVIDIA container tooling (on Manjaro):

```bash
sudo pacman -S nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

4. Test GPU passthrough:

```bash
docker run --rm --gpus all nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi
```

You should see your RTX 4060 listed in the container output.

---

## ✅ Recommended (Docker) installation

### 1) Clone this repository

```bash
git clone https://github.com/SuhanVerse/Learning-ROS2.git
cd Learning-ROS2
```

### 2) Example Dockerfile (optional)

If you want a custom image (base: ubuntu 22.04 + ROS Humble):

```dockerfile
# Dockerfile
FROM ubuntu:22.04
ARG DEBIAN_FRONTEND=noninteractive

# Install basic dependencies and ROS 2 Humble
RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release locales sudo \
    && locale-gen en_US.UTF-8

# Add ROS 2 repo and install (shortened for brevity)
# ... follow official ROS 2 instructions to add keys and apt sources

# Create a user (optional)
RUN useradd -m developer && echo "developer:developer" | chpasswd && adduser developer sudo
USER developer
WORKDIR /home/developer

CMD ["/bin/bash"]
```

> Tip: You can start from `ros:humble` or `ros:humble-ros-base` official images for faster iteration.

### 3) docker-compose.yml (example)

A minimal `docker-compose.yml` for development with GPU support:

```yaml
version: "3.8"
services:
  ros2_dev:
    image: ros:humble
    container_name: ros2_dev
    tty: true
    privileged: true
    network_mode: "host"
    environment:
      - DISPLAY=${DISPLAY}
      - QT_X11_NO_MITSHM=1
    volumes:
      - $HOME/LEARN:/root/LEARN
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
    deploy:
      resources:
        reservations:
          devices:
            - capabilities: [gpu]
    runtime: nvidia
    command: /bin/bash
```

**Start the container:**

```bash
docker compose up -d --build
```

Attach to the container shell:

```bash
docker exec -it ros2_dev bash
```

Inside the container, source ROS 2:

```bash
source /opt/ros/humble/setup.bash
```

---

## 🖥️ Developing with VS Code

You can configure VS Code to open a terminal that drops you into the running container with your workspace sourced. Examples for `.vscode/settings.json` and `.vscode/tasks.json` are shown below.

**Example: `settings.json` terminal profile**

```json
{
  "terminal.integrated.profiles.linux": {
    "ROS2-Docker": {
      "path": "/bin/bash",
      "args": [
        "-c",
        "docker exec -it ros2_dev bash -c '(source /opt/ros/humble/setup.bash && source /root/LEARN/install/setup.bash) || true; exec bash'"
      ]
    }
  },
  "terminal.integrated.defaultProfile.linux": "ROS2-Docker"
}
```

**Example: `tasks.json` snippets for running talker/listener**

```json
{
  "version": "2.0.0",
  "tasks": [
    {
      "label": "run talker",
      "type": "shell",
      "command": "docker exec -it ros2_dev bash -c 'source /opt/ros/humble/setup.bash && source /root/LEARN/install/setup.bash && ros2 run demo_nodes_cpp talker'"
    },
    {
      "label": "run listener",
      "type": "shell",
      "command": "docker exec -it ros2_dev bash -c 'source /opt/ros/humble/setup.bash && source /root/LEARN/install/setup.bash && ros2 run demo_nodes_cpp listener'"
    }
  ]
}
```

> Tip: Use the Remote - Containers extension if you prefer VS Code to manage containers for you.

---

## 🛠 Building your workspace (inside container)

Assuming your ROS 2 workspace is at `/root/LEARN` inside the container:

```bash
# inside container
cd /root/LEARN
colcon build
source install/setup.bash
```

Install extra packages if needed (example):

```bash
apt update && apt install -y ros-humble-turtlesim
```

Run the simulator:

```bash
# Terminal 1
ros2 run turtlesim turtlesim_node

# Terminal 2
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_cpp listener
```

---

## 🔁 Common workflow (quick checklist)

- Start container: `docker compose up -d`
- Attach shell: `docker exec -it ros2_dev bash`
- Source ROS 2 and workspace: `source /opt/ros/humble/setup.bash && source /root/LEARN/install/setup.bash`
- Build: `colcon build`
- Run nodes with `ros2 run` or `ros2 launch`

---

## 🧭 Alternative installation methods on Manjaro

1. **AUR packages** – search for `ros-humble-*` packages in AUR. Good for quick native installs but may require maintenance.
2. **Source build** – follow official ROS 2 instructions to build from source. This is useful if you need bleeding-edge patches or custom core changes.

---

## 🛠 Tips & troubleshooting

- `nvidia-smi` not visible in container: ensure `nvidia-container-toolkit` is installed and `docker run --gpus all` works.
- Permission issues with Docker: add your user to the `docker` group or use `sudo` for Docker commands.
- `DISPLAY` issues for GUI apps: ensure X11 socket is mounted (`/tmp/.X11-unix`) and `xhost +local:root` (or better: more secure alternatives).
- If `colcon build` fails: check package `package.xml` and `CMakeLists.txt` for missing dependencies, and `rosdep` can help install OS dependencies.

---

## 🔗 Resources & further reading

- ROS 2 official docs: [https://docs.ros.org/en/humble/](https://docs.ros.org/en/humble/)
- Colcon build tool: [https://colcon.readthedocs.io/](https://colcon.readthedocs.io/)
- NVIDIA Container Toolkit: [https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/)
- ROS 2 tutorials (beginner-friendly)

---

## 🤝 Contributing

Feel free to open issues or PRs if you have suggestions, updates for newer ROS 2 distributions or better Docker setups.

---

## 📜 License

Licensed under the **Apache-2.0** License.

---
