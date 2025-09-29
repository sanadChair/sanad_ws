# Sanad Workspace (sanad_ws)

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

1. Source the workspace:
```bash
source ~/sanad_ws/install/setup.bash
```

2. Run the node:
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

## Troubleshooting

### Permission Denied when Running Node
```bash
chmod +x ~/sanad_ws/src/sanad_interface_pkg/sanad_interface_pkg/keyboard_teleop.py
```

### Node Not Found
Make sure you've sourced the workspace:
```bash
source ~/sanad_ws/install/setup.bash
```

### Authentication Failed (Clone)
- Ensure you're using your Personal Access Token, not your GitHub password
- Check that your token has `repo` scope enabled
- Verify the token hasn't expired


## Acknowledgments

Built with ROS2 for robotics control and teleoperation.