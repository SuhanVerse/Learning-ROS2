# 📘 Learning ROS 2 & Messaging Concepts — Notes

This document consolidates notes on **ROS 2 basics**, **topics, publishers, subscribers**, and **Docker usage** for ROS 2 Humble.

---

## 📨 Messaging & Topics

### Creating Topics

* Create separate topics for different data streams (e.g., `age_group`, `referral`).
* Example request-style notes (conceptual):

  * `POST /topic/14/11/21/2023`
  * `POST /topic/12/11/21/2023`

### Viewing Topics

* To view input topic:

  ```bash
  /topic/<name>
  ```

## 📡 Publishing & Subscribing

### Topic Publishing (CLI)

* `ros2 topic pub /topic_1111 <type> <data>` — publish a message to a topic.
* `ros2 topic pub -r <rate> /topic_name <type> <data>` — publish repeatedly at a rate.

### Message Echo

* `ros2 topic echo /topic_name` — view messages published to a topic. Useful for debugging and verifying communication.

### Timed Publishing

* Use a **timer** inside a node to publish at a fixed rate.
* For manual testing, `ros2 topic pub -r <rate>` is quick and handy.

## 🔍 Debugging & Visualization

* `ros2 topic list` — list all active topics.
* `ros2 topic info /topic_name` — show details about a topic (type, QoS, publishers, subscribers).
* `ros2 topic hz /topic_name` — check publishing frequency.
* `ros2 topic bw /topic_name` — check bandwidth usage.
* `ros2 topic echo /topic_name` — display messages in real time.
* `rqt_graph` or `ros2 run rqt_graph rqt_graph` — visualize internal communication between nodes (if rqt is installed).

## 🧩 Understanding Nodes

### What is a Node?

* A Node is a program that communicates with other nodes in the ROS 2 network.
* Nodes can:

  * Publish data (Publisher Node)
  * Subscribe to data (Subscriber Node)
  * Provide or call services
  * Serve or call actions

### Node Communication

```
Publisher Node  --->  Topic  --->  Subscriber Node
```

* Publisher Node sends data to a topic.
* Subscriber Node listens to that topic to receive data.

Nodes commonly represent sensors, controllers, planners, or UI components in a robot system.

---

## 🐢 ROS 2 Basics (Humble)

### Installation (Ubuntu 22.04)

```bash
sudo apt update && sudo apt upgrade
sudo apt install ros-humble-desktop
```

### Environment Setup

```bash
source /opt/ros/humble/setup.bash
# make permanent
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Useful Commands

* `ros2 pkg executables <pkg_name>` — list executables in a package.
* `ros2 run <package> <executable>` — run an executable from a package.
* Example: `ros2 run turtlesim turtlesim_node`

---

## 🐳 Docker with ROS 2

### Common Docker Commands

```bash
# list containers
docker ps -a

# start a container
docker start <container_name>

# stop a container
docker stop <container_name>

# remove containers/services started with compose
docker compose down

# run a container interactively
docker run -it osrf/ros:humble-desktop

# run with GUI support
docker run -it --rm --net=host \
  --env="DISPLAY" \
  --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
  osrf/ros:humble-desktop

# follow logs
docker logs -f <container_name>
```

* For GPU support: use `--gpus all` (requires NVIDIA container toolkit and drivers).

---

## ✅ Quick Reference

```bash
# source ROS 2
source /opt/ros/humble/setup.bash

# check nodes
ros2 node list
ros2 node info <node_name>

# check topics
ros2 topic list
ros2 topic echo /topic_name
```

---

## ✍️ Minimal Examples — Python (rclpy)

### talker.py (publisher)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)  # 2 Hz
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'hello {self.count}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### listener.py (subscriber)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Build & run (quick steps)

1. Create a Python package with dependencies:

```bash
ros2 pkg create --build-type ament_python my_pkg --dependencies rclpy std_msgs
```

2. Place `talker.py` and `listener.py` under `my_pkg/my_pkg/` and update `package.xml`/`setup.py`.
3. Build and source:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

4. Run in separate terminals:

```bash
ros2 run my_pkg talker
ros2 run my_pkg listener
```

---

## 🧭 Tips & Troubleshooting

* If `ros2 topic echo` shows nothing, check `ros2 topic list` and `ros2 node list` to verify publishers exist.
* `colcon build` failing? Inspect `package.xml`, `CMakeLists.txt`, and ensure runtime/build dependencies are declared.
* GUI apps not appearing in container: ensure X11 socket mounted (`/tmp/.X11-unix`) and `DISPLAY` forwarded; `xhost +local:root` can help (use with caution).
* GPU not visible in container: verify `nvidia-smi` on host, and test `docker run --gpus all nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi`.

---

## 🔗 Helpful Resources

* ROS 2 docs & tutorials: [https://docs.ros.org/en/humble/](https://docs.ros.org/en/humble/)
* Colcon docs: [https://colcon.readthedocs.io/](https://colcon.readthedocs.io/)
* NVIDIA container toolkit docs: [https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/)

---
