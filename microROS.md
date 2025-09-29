Run this in your terminal to have micro ros fully set up. 

## 🛠️ Setup micro-ROS Agent

### 1. Install dependencies

Make sure your system is up-to-date and has the required ROS 2 packages:

```bash
sudo apt update && sudo apt upgrade -y

# Core ROS 2 tools
sudo apt install -y \
  build-essential git python3-colcon-common-extensions \
  python3-pip python3-vcstool cmake

# micro-ROS dependencies
sudo apt install -y \
  ros-humble-micro-ros-msgs \
  ros-humble-ros2cli \
  ros-humble-ros2cli-common-extensions
```

---

### 2. Clone micro-ROS Agent

```bash
cd ~/ros2_ws/src
git clone -b humble https://github.com/micro-ROS/micro-ROS-Agent.git
```

---

### 3. Build

```bash
cd ~/ros2_ws
colcon build --packages-select micro_ros_agent
```

---

### 4. Source the workspace

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

---

### 5. Run the Agent

Connect your ESP32 via USB, then run:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v6
```

> 🔹 Replace `/dev/ttyUSB0` with the actual port shown by:
>
> ```bash
> ls /dev/ttyUSB*
> ```
