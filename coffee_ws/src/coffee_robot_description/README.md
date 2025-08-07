# Coffee Robot Description Package

A complete 3D robot description package for the Coffee Buddy robot with animatronic head and coffee machine base. This package provides URDF models, TF integration, RViz visualization, and Gazebo simulation capabilities.

## Overview

The `coffee_robot_description` package contains:

- **URDF/Xacro Models**: Complete 3D robot description with coffee machine base, 3DOF neck, animatronic head, LCD display, camera, and movable ears
- **TF Integration**: Real-time coordinate frame publishing that integrates with your existing head control system
- **RViz Visualization**: 3D visualization with real-time head movement tracking
- **Gazebo Simulation**: Physics-based simulation with camera and sensor plugins
- **ROS2 Control**: Controller configurations for head and ear movements

### Current Status (ROS2 Jazzy)

✅ **Working**: URDF models, TF integration, RViz visualization, real-time head tracking  
✅ **Working**: Integration with existing head control system via `/head_pan_angle` and `/head_tilt_angle`  
✅ **Working**: Complete 3D robot model with coffee machine, neck, head, camera, ears  
✅ **Working**: Dual-mode operation (hardware integration + standalone simulation)  
✅ **Working**: Manual joint control GUI for development and testing  
✅ **Ready for Future**: 3rd DOF roll motor and ear actuation (hardware expansion)

## Hardware Integration

### Current Hardware (Implemented)
- **Pan Motor**: Dynamixel XM540-W270 (ID: 1) - Neck yaw movement
- **Tilt Motor**: Dynamixel XM540-W270 (ID: 9) - Neck pitch movement  
- **Camera**: Logitech Brio 4K positioned in forehead above LCD display
- **Display**: LCD screen for facial expressions (middle of head)
- **Coffee Machine**: Delonghi Prima Donna base with full model

### Future Hardware (Planned)
- **Roll Motor**: 3rd DOF for complete neck movement
- **Ear Motors**: Left and right ear actuation (45° range each)
- **Additional Sensors**: IMU, additional cameras

## Quick Start

### Automated Setup (Recommended)

The Coffee Buddy project includes automated setup scripts that handle all dependencies and configuration for both Ubuntu and macOS:

```bash
# Initial setup (run once)
./scripts/setup_workspace.sh

# Daily activation (run in each new terminal)
source scripts/activate_workspace.sh
```

**Platform Support:**
- **Ubuntu/Debian**: Uses native ROS2 with apt + Python virtual environment
- **macOS**: Uses RoboStack with mamba/conda environment

**What the setup script installs:**
- ROS2 Jazzy with all required packages (`xacro`, `joint-state-publisher-gui`, etc.)
- Robot state publisher and visualization tools
- Development tools (colcon, cmake, etc.)
- Python dependencies
- Builds all workspace packages automatically

### Manual Control GUI

Once setup is complete, run the robot description with manual joint control:

```bash
# Activate environment (if not already active)
source scripts/activate_workspace.sh

# Launch with manual control GUI
cd coffee_ws
ros2 launch coffee_robot_description rviz_display.launch.py use_manual_control:=true
```

This opens RViz with the 3D robot model and a GUI with sliders to manually control joint angles.

## Installation

### Automated Setup (Recommended)

**Use the project setup scripts for the easiest installation:**

```bash
# From the project root directory
./scripts/setup_workspace.sh
```

This handles all dependencies, environment setup, and builds all packages automatically for both Ubuntu and macOS.

### Manual Installation (Advanced/Development)

If you prefer manual installation or need to install individual components:

#### Prerequisites

**For Ubuntu/Debian (manual installation):**

```bash
# Core ROS2 dependencies (required)
sudo apt update
sudo apt install ros-jazzy-robot-state-publisher ros-jazzy-xacro ros-jazzy-rviz2

# Joint state publishers (for manual control and development)
sudo apt install ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui

# Optional dependencies (for simulation and advanced features)
sudo apt install ros-jazzy-gazebo-ros-pkgs ros-jazzy-controller-manager ros-jazzy-joint-trajectory-controller
```

**For macOS:**
Use RoboStack with mamba (see automated setup script or RoboStack documentation).

**Note**: The automated setup script (`./scripts/setup_workspace.sh`) handles all these dependencies automatically for both platforms.

**Note**: If you encounter GPG key errors during installation, update your ROS2 repository configuration:
```bash
# Modern repository setup (automatic key management)
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
```

### Build the Package

**Using Automated Setup (Recommended):**
The setup script builds all packages automatically. If you used `./scripts/setup_workspace.sh`, you can skip this section.

**Manual Build (for development/changes):**

```bash
cd coffee_ws

# Build the package
colcon build --packages-select coffee_robot_description

# Source the workspace
source install/setup.bash
```

**Note**: After making changes to URDF files or launch files, you only need to rebuild this specific package using the commands above.

## Usage

**Environment Setup**: Before running any commands, make sure your environment is activated:
```bash
# If using automated setup
source scripts/activate_workspace.sh

# Or manually (if you built manually)
cd coffee_ws && source install/setup.bash
```

The package supports **dual-mode operation** for different use cases:

### Mode 1: Hardware Integration (Default)

Real-time 3D visualization synchronized with your physical robot:

```bash
# Hardware integration mode (default)
ros2 launch coffee_robot_description rviz_display.launch.py

# Or explicitly specify hardware mode
ros2 launch coffee_robot_description rviz_display.launch.py use_manual_control:=false
```

**Features:**
- Integrates with existing head control system
- Subscribes to `/head_pan_angle` and `/head_tilt_angle` topics
- Real-time 3D visualization of actual robot movements
- Perfect for monitoring, debugging, and demonstrations

### Mode 2: Standalone Simulation 

Manual control and algorithm development without hardware dependency:

```bash
# Standalone simulation mode
ros2 launch coffee_robot_description rviz_display.launch.py use_manual_control:=true
```

**Features:**
- Manual joint control sliders in GUI
- Send position commands programmatically
- Algorithm testing and development
- Works completely offline from hardware

### Basic Robot State Publisher

For minimal setups or integration into other launch files:

```bash
# Just robot state publisher (no RViz)
ros2 launch coffee_robot_description robot_state_publisher.launch.py

# With manual control support
ros2 launch coffee_robot_description robot_state_publisher.launch.py use_manual_control:=true
```

### When to Use Each Mode

**Hardware Integration Mode** (`use_manual_control:=false`):
- ✅ **Production operation** - monitoring real robot behavior
- ✅ **Hardware debugging** - visualizing actual motor positions
- ✅ **Demonstrations** - showing real-time robot movement
- ✅ **System integration** - connecting with existing head control

**Standalone Simulation Mode** (`use_manual_control:=true`):
- ✅ **Algorithm development** - testing without hardware
- ✅ **Motion planning** - validating joint trajectories
- ✅ **Remote development** - working away from physical robot
- ✅ **Safe testing** - experimenting without motor wear
- ✅ **Multi-developer** - multiple people working simultaneously

### Programmatic Control (Standalone Mode)

In standalone simulation mode, you can control the robot programmatically:

```bash
# Send joint state commands
ros2 topic pub /joint_states sensor_msgs/msg/JointState "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''},
  name: ['neck_yaw_joint', 'neck_pitch_joint'],
  position: [0.5, -0.2],
  velocity: [],
  effort: []
}"

# Move ears (when available)
ros2 topic pub /joint_states sensor_msgs/msg/JointState "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''},
  name: ['neck_yaw_joint', 'neck_pitch_joint', 'left_ear_joint', 'right_ear_joint'],
  position: [0.0, 0.0, 0.3, -0.3],
  velocity: [],
  effort: []
}"
```

### Integration with Existing Head Control

The hardware integration mode automatically connects to your existing system:

**Automatic Integration:**
- TF publisher subscribes to `/head_pan_angle` and `/head_tilt_angle` topics
- Converts motor coordinates (143-210°, 169-206°) to URDF coordinates (-37° to +30°, -11° to +26°)
- Publishes joint states for robot_state_publisher
- Provides real-time 3D visualization of head movements

**Manual Integration:**
```bash
# Run TF publisher standalone (if needed)
ros2 run coffee_robot_description tf_publisher
```

### 3. Gazebo Simulation

Run the complete robot in Gazebo physics simulation:

```bash
# Launch Gazebo simulation
ros2 launch coffee_robot_description gazebo_sim.launch.py

# With custom world file
ros2 launch coffee_robot_description gazebo_sim.launch.py world:=cafe_world.world

# With custom robot position
ros2 launch coffee_robot_description gazebo_sim.launch.py x_pose:=1.0 y_pose:=2.0
```

### 4. Camera Integration

Access camera feeds from the head-mounted Logitech Brio:

```bash
# In simulation, camera topics will be available:
ros2 topic list | grep camera

# Camera info and image topics:
# /head_camera/image_raw
# /head_camera/camera_info
```

## Package Structure

```
coffee_robot_description/
├── urdf/                              # Robot description files
│   ├── coffee_robot.urdf.xacro       # Main robot assembly
│   ├── coffee_machine_base.urdf.xacro # Coffee machine base
│   ├── neck_assembly.urdf.xacro      # 3DOF neck assembly  
│   ├── robot_head.urdf.xacro         # Head with display, camera, ears
│   └── sensors.urdf.xacro            # Camera and sensor definitions
├── launch/                           # Launch files
│   ├── robot_state_publisher.launch.py # Basic robot visualization
│   ├── rviz_display.launch.py        # RViz with GUI controls
│   └── gazebo_sim.launch.py          # Gazebo simulation
├── config/                           # Configuration files
│   ├── joint_limits.yaml            # Joint limits and safety
│   └── controllers.yaml             # ROS2 control configuration
├── rviz/                            # RViz configurations
│   └── coffee_robot.rviz            # Pre-configured RViz setup
├── meshes/                          # 3D mesh files (for realistic appearance)
│   ├── stl/                         # STL files for visualization
│   └── collada/                     # DAE files for physics
└── coffee_robot_description/        # Python modules
    └── tf_publisher.py              # TF integration node
```

## Coordinate Systems

### Motor Coordinates → URDF Coordinates

The system automatically converts between your existing motor coordinate system and standard URDF conventions:

**Pan Motor (ID: 1)**
- Motor Range: 143° to 210° (center: 180°)
- URDF Range: -37° to +30° (center: 0°)
- Conversion: `urdf_angle = motor_angle - 180°`

**Tilt Motor (ID: 9)**  
- Motor Range: 169° to 206° (center: 180°)
- URDF Range: -11° to +26° (center: 0°)
- Conversion: `urdf_angle = motor_angle - 180°`

### Frame Definitions

```
world
└── base_link (coffee machine)
    └── neck_mount_link
        └── neck_yaw_link (pan motor)
            └── neck_pitch_link (tilt motor)
                └── neck_roll_link (future roll motor)
                    └── head_link (main head body)
                        ├── display_link (LCD screen)
                        │   ├── camera_mount_link (Logitech Brio)
                        │   └── camera_optical_frame (ROS camera standard)
                        ├── left_ear_link (future ear motor)
                        ├── right_ear_link (future ear motor)
                        └── imu_link (future IMU sensor)
```

## Integration Examples

### 1. Add to Existing Launch File

```python
# In your existing launch file
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Include robot description
robot_description_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            FindPackageShare('coffee_robot_description'),
            'launch', 'robot_state_publisher.launch.py'
        ])
    ])
)
```

### 2. Subscribe to Joint States

```python
# Monitor joint states in your nodes
from sensor_msgs.msg import JointState

def joint_state_callback(self, msg):
    # Find neck joints
    try:
        yaw_idx = msg.name.index('neck_yaw_joint')
        pitch_idx = msg.name.index('neck_pitch_joint')
        
        yaw_angle = msg.position[yaw_idx]
        pitch_angle = msg.position[pitch_idx]
        
        # Use joint angles...
    except ValueError:
        pass  # Joint not found
```

### 3. Command Robot Joints

```python
# Send joint trajectory commands
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def move_head(self, yaw, pitch):
    trajectory = JointTrajectory()
    trajectory.joint_names = ['neck_yaw_joint', 'neck_pitch_joint']
    
    point = JointTrajectoryPoint()
    point.positions = [yaw, pitch]
    point.time_from_start = Duration(seconds=2.0)
    
    trajectory.points = [point]
    self.trajectory_publisher.publish(trajectory)
```

## Configuration

### Joint Limits

Modify `config/joint_limits.yaml` to adjust:
- Position limits (based on your hardware)
- Velocity and acceleration limits
- Safety factors
- Emergency stop parameters

### Controllers

Modify `config/controllers.yaml` to configure:
- Update rates
- Trajectory constraints  
- Hardware interface parameters
- Integration settings

## Future Expansion

### Adding Roll Motor (3rd DOF)

When you add the roll motor hardware:

1. **Update Hardware**: Connect roll motor to your Dynamixel chain
2. **Update TF Publisher**: Add subscription to roll angle topic
3. **Update Controllers**: Enable roll joint in controller configuration
4. **No URDF Changes Needed**: The model already includes the roll joint

### Adding Ear Motors

When you add ear actuation hardware:

1. **Update Hardware**: Connect ear motors
2. **Update TF Publisher**: Add ear angle subscriptions  
3. **Update Controllers**: Enable ear joints in controller configuration
4. **No URDF Changes Needed**: The model already includes ear joints

## Troubleshooting

### ROS2 Jazzy Issues

**Import Error for FindPackageShare:**
```
ImportError: cannot import name 'FindPackageShare' from 'launch.substitutions'
```
**Solution**: Fixed in our launch files - uses `launch_ros.substitutions.FindPackageShare`

**Parameter Value Error:**
```
Unable to parse the value of parameter robot_description as yaml
```
**Solution**: Fixed in our launch files - uses `ParameterValue(content, value_type=str)`

**Repository GPG Key Errors:**
```
GPG error: http://packages.ros.org/ros2/ubuntu InRelease: EXPKEYSIG F42ED6FBAB17C654
```
**Solution**: Update to modern repository configuration (see Prerequisites section)

**Package Installation 404 Errors:**
```
404 Not Found for ros-jazzy-joint-state-publisher
```
**Solution**: Usually caused by expired GPG keys. Update repository configuration first, then retry installation.

### No Robot Visible in RViz
```bash
# Check if robot_description topic exists
ros2 topic echo /robot_description

# Check if joint_states are being published
ros2 topic echo /joint_states

# Verify TF tree
ros2 run tf2_tools view_frames
```

### TF Integration Issues
```bash
# Check if head angle topics exist
ros2 topic list | grep head

# Monitor head angles
ros2 topic echo /head_pan_angle
ros2 topic echo /head_tilt_angle

# Check TF publisher status
ros2 node info /coffee_robot_tf_publisher
```

### URDF Parsing Errors
```bash
# Check URDF syntax
check_urdf coffee_ws/install/coffee_robot_description/share/coffee_robot_description/urdf/coffee_robot.urdf.xacro

# Or use xacro to process and check
xacro coffee_ws/src/coffee_robot_description/urdf/coffee_robot.urdf.xacro > /tmp/robot.urdf
check_urdf /tmp/robot.urdf
```

## Contributing

When adding new components:

1. **Follow naming conventions**: Use descriptive link and joint names
2. **Add proper inertial properties**: Ensure realistic mass and inertia
3. **Include collision geometry**: For physics simulation
4. **Update configuration files**: Add joint limits and controller configs
5. **Test in both RViz and Gazebo**: Verify visualization and simulation

## Dependencies

### Required (Core Functionality)
- ROS2 Jazzy (or compatible)
- robot_state_publisher
- xacro
- rviz2
- tf2_ros

### Required (Dual-Mode Operation)
- joint_state_publisher (for standalone simulation)
- joint_state_publisher_gui (for manual control interface)

### Optional (Advanced Features)
- gazebo_ros (for physics simulation)
- controller_manager (for ROS2 control)
- joint_trajectory_controller (for trajectory control)

**Note**: All packages are now available with proper ROS2 repository configuration. See Prerequisites section for installation details.

## License

[TODO: Add your license information] 