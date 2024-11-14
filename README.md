```
colcon build --packages-select fp-mantap
source /usr/share/gazebo/setup.sh
source install/setup.bash
ros2 launch fp-mantap launch.py
```
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 topic pub /target_position geometry_msgs/msg/Point "{x: 5.0, y: 5.0, z: 0.0}"

```
