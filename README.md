# Part 1: Sanad Workspace (sanad_ws)

A ROS2 workspace containing the `sanad_interface_pkg` package for robot control interfaces.

## Prerequisites

Before cloning this project, you need to create a GitHub Personal Access Token to authenticate.

### Creating a GitHub Personal Access Token

1. Log in to your GitHub account
2. Click on your profile picture (top right) → **Settings**
3. Scroll down and click **Developer settings** (left sidebar, bottom)
4. Click **Personal access tokens** → **Tokens (classic)**
5. Click **Generate new token** → **Generate new token (classic)**
6. Give your token a descriptive name (e.g., "ROS2 Sanad Workspace")
7. Set expiration date (recommended: 90 days or custom)
8. Select scopes: Check **repo** (this gives full control of private repositories)
9. Scroll down and click **Generate token**
10. **IMPORTANT**: Copy the token immediately and save it securely (you won't be able to see it again)

## Installation

### 1. Clone the Repository

```bash
cd ~
git clone https://github.com/sanadChair/ROS.git
cd sanad_ws
```

When prompted for credentials:
- **Username**: Your GitHub username
- **Password**: Paste your Personal Access Token (not your GitHub password)



### 2. Build the Workspace

```bash
cd ~/sanad_ws
colcon build --symlink-install # useful for faster iteration on Python packages.
source install/setup.bash
```
This command will create the build/, install/, and log/ directories.


## Running the Packages

### Running the Keyboard Teleop Node

1.Source the workspace:

```bash
source ~/sanad_ws/install/setup.bash
```

2.Run the node:

```bash
ros2 run sanad_interface_pkg <your_node_executable_name>
```

## Development

### Adding New Nodes

1. Create your node in `src/sanad_interface_pkg/sanad_interface_pkg/`
2. Make it executable: `chmod +x your_node.py`
3. Add entry point in `setup.py`
4. Rebuild: `colcon build --packages-select sanad_interface_pkg`

### Contributing

1. Create a new branch: `git checkout -b feature/your-feature`
2. Make your changes
3. Commit: `git commit -am 'Add new feature'`
4. Push: `git push origin feature/your-feature`
5. Create a Pull Request



# Part 2: Sanad Chair Control System

This repository contains the firmware for the **Sanad Chair**, a smart chair controlled remotely via **micro-ROS** over Wi-Fi. The system is designed for an **ESP32** or similar microcontroller, utilizing the **PlatformIO** development ecosystem.

## 🗃️ Project Structure

The project is organized using PlatformIO's standard structure, with custom libraries stored in the `lib` folder.

``` 
sanad_control/
├── .pio/               # PlatformIO build files
├── .vscode/            # VS Code configuration
├── include/            # Common header files (if any)
├── lib/                # Custom Libraries/Classes
│   ├── Config/         # System configuration (pins, network settings)
│   │   └── config.h
│   ├── l298n/          # L298N Motor Driver Control
│   │   ├── l298n.cpp
│   │   └── l298n.h
│   ├── RosComm/        # micro-ROS Communication Node
│   │   ├── RosRobotNode.cpp
│   │   └── RosRobotNode.h
│   ├── sanadChair/     # Top-Level System Class
│   │   ├── sanadChair.cpp
│   │   └── sanadChair.h
│   └── ultrasonic/     # Ultrasonic Sensor Interface
│       ├── ultrasonic.cpp
│       └── ultrasonic.h
├── src/                # Main execution code (Arduino sketch)
│   └── main.cpp
└── README.md           # This file`
```

---

## ⚙️ System Overview

The Sanad Chair system is built around several decoupled components, communicating with a central **ROS 2** network (via a ROS Agent) to achieve remote control and data reporting.

| Component | Responsibility | Files |
| --- | --- | --- |
| **L298N** | Low-level control for the two motors (movement, speed, direction). | `l298n.*` |
| **Ultrasonic Sensor** | Provides distance measurements for obstacle detection. | `ultrasonic.*` |
| **RosRobotNode** | Manages the micro-ROS connection, handles Wi-Fi, subscribes to control commands, and publishes sensor data. | `RosRobotNode.*` |
| **SanadChair** | The top-level class that instantiates, initializes, and manages the lifecycle of the Motor Driver, Sensor, and ROS Node. | `sanadChair.*` |
| **Main Application** | Initializes the `SanadChair` object and calls the main `update()` loop. | `main.cpp` |
| **Config** | Defines all hardware pin assignments (e.g., `TRIG_PIN`, `IN1`), network credentials (`WIFI_SSID`), and ROS Agent connection details. | `config.h` |


---

## 🚀 Key Features

- **Remote Control via ROS 2:** The robot is controlled by receiving movement commands over a **ROS topic** (subscribed by `RosRobotNode`).
- **Decoupled Hardware Interfaces:** The motor driver and sensor logic are encapsulated in their own classes, making them reusable and easy to test.
- **Distance Sensing:** The ultrasonic sensor provides real-time data for navigation and collision avoidance, published to a **ROS topic**.
- **Object-Oriented Architecture:** Clean design using C++ classes to manage state and complexity.
- **PlatformIO Ready:** Designed for easy build, upload, and dependency management using the PlatformIO ecosystem.


---

## 🔌 Setup and Configuration

1. **PlatformIO:** Ensure you have PlatformIO installed in VS Code.
2. **Configuration:** Edit `lib/Config/config.h` to define:
    - The GPIO pins for the **L298N** module (IN1, IN2, IN3, IN4, ENA, ENB).
    - The GPIO pins for the **Ultrasonic Sensor** (TRIG_PIN, ECHO_PIN).
    - Your Wi-Fi SSID and password (`WIFI_SSID`, `WIFI_PASSWORD`).
    - The IP address and port of your ROS Agent (`AGENT_IP`, `AGENT_PORT`).
3. **Dependencies:** PlatformIO will automatically handle external libraries, but ensure the `micro_ros_arduino` library is correctly configured.
4. **ROS Agent:** A ROS Agent must be running on your network (e.g., on a desktop computer) to bridge communication between the microcontroller and the ROS 2 network.

---

## 💻 Usage (Control Flow)

1. The system starts in `main.cpp` by creating a `SanadChair` instance.
2. `chair->begin(...)` is called, which initializes all hardware interfaces, the Wi-Fi connection, and the **micro-ROS node**.
3. The `loop()` function repeatedly calls `chair->update()`, which executes the **ROS executor's spin function** (`_rosNode->spin()`).
4. **Control Input:** When a new movement command (e.g., an integer) arrives on the ROS subscriber topic (e.g., /cmd_vel), the static subscription Callback in `RosRobotNode` translates the message into a motor action (e.g., `_l298n->moveForward()`).
5. **Sensor Output:** A ROS Timer periodically triggers the static `timerCallback`, which reads the distance using `_ultrasonic->getDistance()` and **publishes** the data to the ROS network.




