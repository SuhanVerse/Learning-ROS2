# 📘 ROS 2 Basics & Messaging — Study Notes

A structured guide to understanding **ROS 2 (Humble)** fundamentals — including **nodes**, **topics**, **publishers/subscribers**, **remapping**, and **Docker setup** — organized in the recommended order of learning.

---

## 1️⃣ ROS 2 Installation & Environment Setup

### Installation (Ubuntu 22.04)

```bash
sudo apt update && sudo apt upgrade
sudo apt install ros-humble-desktop
```

### Environment Setup

```bash
source /opt/ros/humble/setup.bash
# Make permanent
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Verify Installation

```bash
ros2 --version
ros2 run demo_nodes_cpp talker
```

---

## 2️⃣ Understanding Nodes

### What is a Node?

A **Node** is an independent process that performs computation in a ROS 2 graph. Nodes communicate through **topics**, **services**, and **actions**.

```text
Publisher Node  --->  Topic  --->  Subscriber Node
```

Each node can:

- Publish data (e.g., from a sensor)
- Subscribe to data (e.g., to control a motor)
- Provide or call services
- Perform tasks in coordination with others

🔗 **Related diagrams:**

- [Single Publisher and Subscriber](https://docs.ros.org/en/humble/_images/Topic-SinglePublisherandSingleSubscriber.gif)
  ![Single Publisher and Subscriber](https://docs.ros.org/en/humble/_images/Topic-SinglePublisherandSingleSubscriber.gif)

---

- [Multiple Publishers & Subscribers](https://docs.ros.org/en/humble/_images/Topic-MultiplePublisherandMultipleSubscriber.gif)
  ![Multiple Publishers & Subscribers](https://docs.ros.org/en/humble/_images/Topic-MultiplePublisherandMultipleSubscriber.gif)

---

## 3️⃣ Topics & Messaging System

### Listing and Inspecting Topics

```bash
ros2 topic list             # show all active topics
ros2 topic info /topic_name # show details (type, QoS, publishers, subscribers)
```

### Viewing Topic Data

```bash
ros2 topic echo /topic_name # view published messages
```

### Publishing Messages

```bash
ros2 topic pub /topic_name <msg_type> '{data}'
ros2 topic pub -r 1 /topic_name <msg_type> '{data}'  # publish at 1 Hz
```

### Monitoring Topics

```bash
ros2 topic hz /topic_name  # check publishing frequency
ros2 topic bw /topic_name  # check bandwidth usage
ros2 topic find <type>     # find topics of a specific message type
```

#### Example — Check topic rate and bandwidth

```bash
ros2 topic hz /turtle1/pose
ros2 topic bw /turtle1/pose
```

---

## 4️⃣ Remapping (2.1)

Remapping lets you change default node or topic names at runtime without editing source code.

Example — renaming a node:

```bash
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
```

Now, running `ros2 node list` shows:

```text
/my_turtle
/turtlesim
/teleop_turtle
```

Remapping is useful when running multiple instances of the same node or avoiding naming conflicts.

---

## 5️⃣ Debugging & Visualization

- `ros2 node list` — list all running nodes
- `ros2 node info <node>` — get details about a node
- `rqt_graph` — visualize node-topic connections
- `ros2 topic echo /topic_name` — observe message data
- `ros2 topic hz /topic_name` — verify publishing rate

These tools are key for debugging publisher/subscriber connectivity.

---

## 6️⃣ Using Docker with ROS 2

### Common Docker Commands

```bash
docker ps -a                 # list containers
docker start <name>          # start container
docker stop <name>           # stop container
docker compose down          # remove containers
docker run -it osrf/ros:humble-desktop # interactive run
```

### Run ROS 2 GUI Apps (with X11)

```bash
docker run -it --rm --net=host \
  --env="DISPLAY" \
  --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
  osrf/ros:humble-desktop
```

Add `--gpus all` for GPU support if available.

---

## 7️⃣ Clean Up & Exit

Stop all running nodes:

```bash
Ctrl + C  # in each terminal
```

Stop Docker containers:

```bash
docker compose down
docker stop <container_name>
```

---

## 🔗 Helpful References

- ROS 2 Docs: [https://docs.ros.org/en/humble/](https://docs.ros.org/en/humble/)
- NVIDIA Container Toolkit: [https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/)
