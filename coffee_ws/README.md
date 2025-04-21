Commands to launch all of the nodes:

```
source ../ros-source.sh
colcon build
ros-source
ros2 launch coffee_head all_nodes.launch.py
```

- Camera node handles opening the camera, tracking faces (`src/coffee_head/coffee_head/camera_node.py`).
- Head tracking handles the PID controller, and is in coordination with the camera node to move the motors to center the detected face in frame (`src/coffee_head/coffee_head/head_tracking.py`).
  - Subscribes to the camera node.
  - Coffee Eyes shows the eye visuals for the Coffee Buddy robot,  (`src/coffee_face/coffee_face/coffee_eyes.py`).
  - Subscribes to the camera node to adjust eye position in window.

Commands to launch separate nodes:

```
source ../ros-source.sh
colcon build
ros-source
ros2 run coffee_head camera_node
ros2 run coffee_head head_tracking
ros2 run coffee_head eye_tracking
ros2 run coffee_face coffee_eyes
```

Running the animator
- Animations are saved in `~/.ros/motion_files` as `.json` files.