cd ~/sanad/ROS/sanad_ws
rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch sanad_bringup sanad_gazebo.launch.xml