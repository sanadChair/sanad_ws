# Orbbec Astra Pro — Setup Guide
### OpenNI SDK v2.3.0.86 + ROS2 Humble on Ubuntu 22.04

---

## Prerequisites

- Ubuntu 22.04 LTS (x86_64)
- ROS2 Humble installed and sourced
- Orbbec Astra Pro camera (USB 3.0 port recommended)
- Internet connection

---

## Part 1 — OpenNI SDK Setup 

### 1.1 Download the SDK

Download the release from GitHub:

**[OpenNI SDK v2.3.0.86-beta6 for Linux](https://github.com/orbbec/OpenNI_SDK/releases/tag/v2.3.0.86-beat6)**

Download the Linux x64 archive, unzip it at ~/OpenNi. This guide assumes:
```
~/OpenNi/OpenNI_2.3.0.86_202210111154_4c8f5aa4_beta6_linux/
```

Expected folder structure after extraction:
```
├── rules/
│   ├── install.sh
│   └── orbbec-usb.rules
├── samples/
│   ├── bin/          ← pre-built binaries + .so libraries
│   └── samples/      ← source code
├── sdk/
│   ├── Include/
│   ├── libs/
│   └── thirdpart/
└── tools/
    └── NiViewer/
```

---

### 1.2 Install System Dependencies

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  freeglut3 \
  freeglut3-dev \
  libusb-1.0-0-dev \
  libuvc-dev
```

---

### 1.3 Fix Missing libpng12 (not in modern Ubuntu repos)

The pre-built SDK binaries were compiled against `libpng12`, which was removed from Ubuntu 18.04+. Install it manually:

```bash
cd /tmp

# Download the .deb from Ubuntu 16.04 archive
wget http://mirrors.kernel.org/ubuntu/pool/main/libp/libpng/libpng12-0_1.2.54-1ubuntu1_amd64.deb

# Extract without installing (avoids usrmerge conflict on Ubuntu 22.04)
dpkg-deb -x libpng12-0_1.2.54-1ubuntu1_amd64.deb libpng12_extracted

# Copy the .so files manually
sudo cp /tmp/libpng12_extracted/lib/x86_64-linux-gnu/libpng12.so.0* /usr/local/lib/

# Refresh linker cache
sudo ldconfig

# Verify
ldconfig -p | grep libpng12
# Expected: libpng12.so.0 (libc6,x86-64) => /usr/local/lib/libpng12.so.0
```

> **Note:** The warning `/usr/local/lib/libpng12.so.0 is not a symbolic link` from ldconfig is harmless — the library is still found correctly.

---

### 1.4 Install Udev Rules

Udev rules give your user account access to the camera's USB device without requiring `sudo`.

```bash
cd ~/OpenNi/OpenNI_2.3.0.86_202210111154_4c8f5aa4_beta6_linux/rules

# Run the install script
sudo ./install.sh

# The Astra Pro exposes TWO USB interfaces:
#   2bc5:0403  →  depth/IR sensor   (covered by the default rules)
#   2bc5:0501  →  HD color camera   (must be added manually)

# Add the missing rule for the color camera
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", ATTR{idProduct}=="0501", MODE="0666", GROUP="video"' \
  | sudo tee -a /etc/udev/rules.d/558-orbbec-usb.rules

# Reload udev
sudo udevadm control --reload-rules
sudo udevadm trigger
```

**Unplug the camera, wait 3 seconds, then plug it back in.**

---

### 1.5 Add Your User to the Required Groups

```bash
sudo usermod -aG video,plugdev $USER
```

Either **log out and back in**, or activate the group in your current terminal:
```bash
newgrp video
```

---

### 1.6 Verify Camera is Detected

```bash
lsusb | grep -i orbbec
```

Expected output (both lines must appear):
```
Bus 001 Device XXX: ID 2bc5:0403 Orbbec 3D Technology International, Inc Astra Pro
Bus 001 Device XXX: ID 2bc5:0501 Orbbec 3D Technology International, Inc Astra Pro HD Camera
```

Check device permissions (replace `001/XXX` with your actual bus/device numbers from lsusb):
```bash
ls -la /dev/bus/usb/001/<device_number_of_0501>
# Expected: crw-rw-rw- 1 root video ...
```

---

### 1.7 Test with SimpleViewer

```bash
cd ~/OpenNi/OpenNI_2.3.0.86_202210111154_4c8f5aa4_beta6_linux/samples/bin

# Make the library path available
export LD_LIBRARY_PATH=$(pwd):$LD_LIBRARY_PATH

# Check for any remaining missing libraries
ldd ./SimpleViewer | grep "not found"
# Should print nothing — all libs found

# Run the viewer
./SimpleViewer
```

A window should open showing the depth stream from the camera.

To make `LD_LIBRARY_PATH` permanent across terminals:
```bash
echo 'export LD_LIBRARY_PATH=~/OpenNi/OpenNI_2.3.0.86_202210111154_4c8f5aa4_beta6_linux/samples/bin:$LD_LIBRARY_PATH' \
  >> ~/.bashrc
source ~/.bashrc
```

---

## Part 2 — ROS2 Humble Integration

### 2.1 Install ROS2 Dependencies

```bash
sudo apt install -y \
  libgflags-dev \
  libgoogle-glog-dev \
  libusb-1.0-0-dev \
  libeigen3-dev \
  libuvc-dev \
  ros-humble-image-geometry \
  ros-humble-camera-info-manager \
  ros-humble-image-transport \
  ros-humble-image-publisher \
  ros-humble-depth-image-proc \
  ros-humble-image-pipeline \
  ros-humble-diagnostic-updater
```

---

### 2.2 Create Workspace and Clone the Driver

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/orbbec/ros2_astra_camera.git
```

---

### 2.3 Copy OpenNI2 Libraries into the Package

The ROS2 driver expects OpenNI2 libraries in its `openni2_redist/` folder:

```bash
SDK_BIN=~/OpenNi/OpenNI_2.3.0.86_202210111154_4c8f5aa4_beta6_linux/samples/bin
REDIST=~/ros2_ws/src/ros2_astra_camera/astra_camera/openni2_redist

# Copy the main OpenNI2 shared library
cp $SDK_BIN/libOpenNI2.so $REDIST/

# Copy the drivers folder (contains liborbbec.so and libOniFile.so)
cp -r $SDK_BIN/OpenNI2 $REDIST/
```

Verify the structure:
```bash
ls $REDIST/
# Expected: libOpenNI2.so  OpenNI2/  OpenNI.ini  (and other existing files)

ls $REDIST/OpenNI2/Drivers/
# Expected: libOniFile.so  liborbbec.so  OniFile.ini  orbbec.ini
```

---

### 2.4 Build the Workspace

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

On success you should see:
```
Summary: 2 packages finished
```

Source the workspace:
```bash
source ~/ros2_ws/install/setup.bash

# Add to .bashrc to auto-source in every terminal
echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
```

---

### 2.5 Launch the Camera Node

```bash
ros2 launch astra_camera astra_pro.launch.xml
```

Or with depth-to-color registration enabled:
```bash
ros2 launch astra_camera astra_camera.launch.xml \
  camera_name:=astra \
  depth_registration:=true
```

---

### 2.6 Verify Topics

In a second terminal:
```bash
source ~/ros2_ws/install/setup.bash
ros2 topic list | grep astra
```

Expected topics:
```
/astra/color/image_raw
/astra/color/camera_info
/astra/depth/image_raw
/astra/depth/camera_info
/astra/depth/points
/astra/ir/image_raw
```

Check data is flowing:
```bash
ros2 topic hz /astra/depth/image_raw
# Expected: ~30 Hz
```

---

### 2.7 Visualize in RViz2

```bash
rviz2
```

Add the following displays manually, or load the provided rviz config if available:

| Display Type | Topic | Notes |
|---|---|---|
| Image | `/astra/color/image_raw` | RGB stream |
| Image | `/astra/depth/image_raw` | Depth stream |
| PointCloud2 | `/astra/depth/points` | Set Fixed Frame to `camera_link` |

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---|---|---|
| `error while loading shared libraries: libOpenNI2.so` | `LD_LIBRARY_PATH` not set | Run `export LD_LIBRARY_PATH=<sdk>/samples/bin:$LD_LIBRARY_PATH` |
| `error while loading shared libraries: libpng12.so.0` | libpng12 not installed | Follow Section 1.3 |
| `uvc_open: Access denied (-3)` | Udev rules missing for PID 0501 | Follow Section 1.4, replug camera |
| `camera init failed` (even with sudo) | Udev rule for 0501 not applied yet | Unplug and replug after reloading udev |
| `lsusb` shows only one Orbbec device | Camera not fully initialized | Try a different USB port (USB 3.0 required) |
| `libuvc is not found` (colcon build) | libuvc-dev not installed | `sudo apt install libuvc-dev` |
| No topics after launch | OpenNI2 drivers not in `openni2_redist/` | Redo Section 2.3 |
| RViz shows no point cloud | Wrong Fixed Frame | Set Fixed Frame to `camera_link` in RViz Global Options |

---

## Quick Reference — Full Command Sequence

For a fresh machine, run these in order:

```bash
# 1. System deps
sudo apt install -y build-essential freeglut3 freeglut3-dev libusb-1.0-0-dev libuvc-dev \
  libgflags-dev libgoogle-glog-dev libeigen3-dev \
  ros-humble-image-geometry ros-humble-camera-info-manager ros-humble-image-transport \
  ros-humble-image-publisher ros-humble-depth-image-proc ros-humble-image-pipeline \
  ros-humble-diagnostic-updater

# 2. libpng12
cd /tmp && wget http://mirrors.kernel.org/ubuntu/pool/main/libp/libpng/libpng12-0_1.2.54-1ubuntu1_amd64.deb
dpkg-deb -x libpng12-0_1.2.54-1ubuntu1_amd64.deb libpng12_extracted
sudo cp /tmp/libpng12_extracted/lib/x86_64-linux-gnu/libpng12.so.0* /usr/local/lib/
sudo ldconfig

# 3. Udev rules  (from SDK rules/ folder)
sudo ./install.sh
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", ATTR{idProduct}=="0501", MODE="0666", GROUP="video"' \
  | sudo tee -a /etc/udev/rules.d/558-orbbec-usb.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# 4. User groups
sudo usermod -aG video,plugdev $USER && newgrp video

# 5. ROS2 workspace
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/orbbec/ros2_astra_camera.git
SDK_BIN=~/OpenNi/OpenNI_2.3.0.86_202210111154_4c8f5aa4_beta6_linux/samples/bin
REDIST=~/ros2_ws/src/ros2_astra_camera/astra_camera/openni2_redist
cp $SDK_BIN/libOpenNI2.so $REDIST/ && cp -r $SDK_BIN/OpenNI2 $REDIST/

# 6. Build
cd ~/ros2_ws && source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 7. Launch
ros2 launch astra_camera astra_pro.launch.xml
```

---

*Tested on Ubuntu 22.04 LTS, ROS2 Humble, OpenNI SDK v2.3.0.86-beta6, Orbbec Astra Pro.*
