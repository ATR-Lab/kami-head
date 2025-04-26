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
- Coffee Expression show the latest version of the eye shapes with a new topic message (`src/coffee_expressions/coffee_expressions/plaipin_expressive_eyes.py`)
- Coffee Eyes shows the eye visuals for the Coffee Buddy robot,  (`src/coffee_face/coffee_face/coffee_eyes.py`).
  - Subscribes to the camera node to adjust eye position in window.expressions_test_ui
- Coffee Expressions Test UI shows the latest version of the eye shapes with a new topic message (`src/coffee_expressions_state_ui/coffee_expressions_state_ui/coffee_expressions_state_ui.py`).
- Coffee Expressions State Manager handles the state of the robot's expressions (`src/coffee_expressions_state_manager/coffee_expressions_state_manager/state_manager_node.py`).
- Coffee Expressions State UI shows the state of the robot's expressions (`src/coffee_expressions_state_ui/coffee_expressions_state_ui/coffee_expressions_state_ui.py`).


# Topics

- `/vision/emotion` - Vision emotion   

- `/voice/intent` - Voice intent   

- `/vision/face_position` - Face position -- the position of the face in the frame of the camera viewer. NOTE that we can dynamically update the field of view (FOV) of the camera viewer. The smaller the FOV, the faster the head tracking will be.   

- `/system/event` - System event   

- `/robot/affective_state` - Affective state -- the aggregation of the state of the robot's expressions.   

- `/robot/state_manager/diagnostics` - Diagnostics

Commands to launch separate nodes:

```
source ../ros-source.sh
colcon build
ros-source
ros2 run coffee_head camera_node
ros2 run coffee_head head_tracking
ros2 run coffee_head eye_tracking
ros2 run coffee_expressions plaipin_expressive_eyes
ros2 run coffee_face coffee_eyes
ros2 run coffee_expressions_test_ui expressions_test_ui
ros2 launch coffee_expressions_state_manager state_manager.launch.py

ros2 run coffee_expressions_state_ui state_ui
```

**NOTE:** We define the location of `expressions.json` in the `setup.py` file inside `coffee_expressions` directory.

Running the animator
- Animations are saved in `~/.ros/motion_files` as `.json` files.


# Sample Command

```
src $ ros2 pkg create --build-type ament_python coffee_expressions_test_ui --dependencies rclpy coffee_expressions_msgs python3-pyqt5
```

```
colcon build --packages-select coffee_expressions_test_ui
```

```
source install/setup.bash && ros2 run coffee_expressions_test_ui expressions_test_ui
```


```
ros2 topic echo /robot/affective_state --field gaze_target_v2
```