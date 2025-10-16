**[Working ROS Task Video](https://drive.google.com/file/d/162Ii2pX5PmuE5KjTJ5nrahzZ9EUmkqlG/view?usp=drive_link)**

````markdown
# micro-ROS on Host Setup Guide 

This guide provides step-by-step instructions for setting up a micro-ROS workspace, building the tools, and running a sample application on your host machine. This allows you to develop and test micro-ROS applications without needing physical hardware.

---

##  1. Setup the micro-ROS Workspace

First, we'll create a dedicated workspace, download the necessary setup tools, and build them.

### Source ROS 2
Ensure your ROS 2 environment is sourced. Replace `$ROS_DISTRO` with your installed ROS 2 version (e.g., `humble`, `iron`).
```bash
# Source the ROS 2 installation
source /opt/ros/$ROS_DISTRO/setup.bash
````

### Create Workspace and Download Tools

Create a directory for your workspace and clone the `micro_ros_setup` repository.

```bash
# Create a workspace and download the micro-ROS tools
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO [https://github.com/micro-ROS/micro_ros_setup.git](https://github.com/micro-ROS/micro_ros_setup.git) src/micro_ros_setup
```

### Install Dependencies

Update your package lists and use `rosdep` to install all required dependencies.

```bash
# Update dependencies using rosdep
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

# Install pip
sudo apt-get install python3-pip
```

### Build and Source the Tools

Build the packages in the workspace and source the new setup file to make them available in your terminal.

```bash
# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash
```

>  **Success\!** You now have a workspace with a ready-to-use micro-ROS build system.

-----

##  2. Understanding the Build System Workflow

The build system simplifies the process of developing for micro-ROS with a four-step procedure:

1.  **Create step**: Downloads the required source code, cross-compilation toolchains, and a collection of ready-to-use micro-ROS apps for a specific hardware platform.
2.  **Configure step**: Allows you to select which app to compile and configure settings like transport protocol and agent connection details (IP address, port, etc.).
3.  **Build step**: Cross-compiles the source code to generate platform-specific binaries.
4.  **Flash step**: Flashes the generated binaries onto the target hardware's memory.

-----

##  3. Create and Build the Firmware

Now, let's use the build system to prepare our application firmware.

### Create the Firmware Workspace

This command downloads the source code and tools needed for your target platform. In this case, the target is the `host` machine itself.

```bash
# Create firmware step
ros2 run micro_ros_setup create_firmware_ws.sh host
```

After running this, a new `firmware` directory will appear in your workspace. Inside `firmware/src/uros/micro-ROS-demos/rclc`, you'll find example applications.

### The `ping_pong` Example

For this tutorial, we will use the `ping_pong` application. This example demonstrates a micro-ROS node that uses a publisher/subscriber pair to communicate.

The node sends a `ping` message. If an external ROS 2 node sends a `fake_ping` message, the micro-ROS node receives it and responds with a `pong` message.

### Build the Firmware

Now, we compile the firmware.

> **Note:** Because we are compiling for the host machine and not cross-compiling for different hardware, the **Configure step** is not needed and can be skipped.

```bash
# Build step
ros2 run micro_ros_setup build_firmware.sh
source install/local_setup.bash
```

-----

##  4. Create and Build the micro-ROS Agent

The micro-ROS agent acts as a bridge, connecting your micro-ROS application to the main ROS 2 network (DDS).

### Create the Agent Workspace

Download the agent packages using the setup script.

```bash
# Download micro-ROS-Agent packages
ros2 run micro_ros_setup create_agent_ws.sh
```

### Build and Source the Agent

Build the agent packages and source the installation.

```bash
# Build step
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

-----

##  5. Running the Application

You're all set\! Let's run the agent and the application to see them communicate.

### Step 1: Run the micro-ROS Agent

Open a new terminal and run the following command to start the agent. The agent will listen for connections from micro-ROS clients on UDP port 8888.

```bash
# Run a micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

### Step 2: Run the micro-ROS App

Open a **second terminal**. Navigate to your workspace, source the environment, and run the `ping_pong` application you built earlier.

```bash
# Navigate to your workspace
cd ~/microros_ws

# Source the environment
source install/local_setup.bash

# Run the ping_pong app (the exact command might vary)
ros2 run micro_ros_demos_rclc ping_pong
```

You should now see output in both terminals, indicating that the `ping_pong` app has successfully connected to the agent and is communicating with the ROS 2 network.

-----

## 💡 Optional(not optional): Add to `.bashrc`

To avoid sourcing the setup files in every new terminal, you can add them to your `.bashrc` file.

```bash
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
echo "source ~/microros_ws/install/local_setup.bash" >> ~/.bashrc
```

You will need to restart your terminal or run `source ~/.bashrc` for the changes to take effect.

```
```
