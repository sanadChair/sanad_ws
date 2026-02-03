# Part 1: Sanad Workspace (sanad_ws)

1
A ROS2 workspace containing the `sanad_interface_pkg` package for robot control interfaces.

## Prerequisites

Before cloning this project, you need to create a GitHub Personal Access Token to authenticate.

<details>
<summary><b>  Creating a GitHub Personal Access Token</b></summary>

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

</details>

## Installation

### 1. Clone the Repository

```bash
cd ~
git clone https://github.com/sanadChair/sanad_ws.git
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

**Optional:** To automatically source the workspace setup on every new terminal session, add the following line to your `.bashrc`:

```bash
echo "source ~/sanad_ws/install/setup.bash" >> ~/.bashrc
```

Then reload your shell:

```bash
source ~/.bashrc
```

## Running the Packages

#### 1. Gazebo Simulation (Gazebo Plugin)

To start the robot simulation in Gazebo with `gz_ros2_control` (loads `small_house.world` by default):

```bash
ros2 launch sanad_bringup sanad_gazebo.launch.py
```

To load the empty world instead:

```bash
ros2 launch sanad_bringup sanad_gazebo.launch.py world_name:=empty
```

To control the robot using stamped teleop keyboard:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel -p stamped:=true
```

#### 2. Real Hardware (DiffDrive Arduino - Default)

To launch with the real Arduino-based differential drive hardware:

```bash
ros2 launch sanad_bringup sanad.launch.py
```

To control the robot using stamped teleop keyboard:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard -p stamped:=true
```

#### 3. Mock Hardware (Testing)

To launch with mock hardware for testing without physical hardware:

```bash
ros2 launch sanad_bringup sanad.launch.py ros2_control_hardware:=mock
```

To control the robot using stamped teleop keyboard:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard -p stamped:=true
```

**Note:** The `stamped:=true` parameter enables stamped twist messages, which provide better timestamp information for the control system. The remapping in the Gazebo version ensures commands are properly routed to the diff_drive_controller.

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

### ⚙️ Sensor-to-Software Mapping Table

| **Sensor / Input** | **ROS2 Node / Package**        | **Published Topics** | **Msg Type**       | **Rate (Hz)** | **Processor** | **Purpose / Notes**                                                          |
| ------------------ | ------------------------------ | -------------------- | ------------------ | ------------- | ------------- | ---------------------------------------------------------------------------- |
| **keyboard**       | `keyboard/sanad_interface_pkg` | `/cmd_vel`           | `sensor_msgs/Int8` | \_            | \_            | Publish keyboard commands throught Micro Ros agent to control sanad movement |
