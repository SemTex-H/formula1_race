pip install "numpy<2"
cd ros2_ws
colcon build
source install/setup.bash
ros2 launch fastbot_racing start_racing.launch.py