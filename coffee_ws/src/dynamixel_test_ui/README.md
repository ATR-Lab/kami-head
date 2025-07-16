# Dynamixel Test UI

A ROS2 package for testing and debugging Dynamixel servos with a Python-based GUI.

## Virtual Environment Setup

Due to permission issues on mounted drives, you might encounter problems when creating a virtual environment. Here are different approaches to try:

### Option 1: Create a virtual environment in your home directory
```bash
# Create the virtual environment in your home directory
python3 -m venv ~/coffee_python_env

# Activate the virtual environment
source ~/coffee_python_env/bin/activate

# Install required dependencies
pip install -e .
pip install -r requirements.txt
```

### Option 2: Use virtualenv package
```bash
# Install virtualenv if not already installed
pip3 install --user virtualenv

# Create the virtual environment
python3 -m virtualenv coffee_python_env

# Activate the virtual environment
source coffee_python_env/bin/activate

# Install required dependencies
pip install -e .
pip install -r requirements.txt
```

### Option 3: Use the system Python with ROS
If virtual environments are problematic, you can use the ROS Python environment directly:
```bash
# Source your ROS environment
source /opt/ros/<your_ros_distro>/setup.bash

# Install dependencies using rosdep
rosdep install --from-paths . --ignore-src -y

# Or install the Python packages to your user site packages
pip3 install --user -r requirements.txt
```

## Running the Package

After setting up your environment, build and run the package:
```bash
# Build the package
cd /path/to/your/workspace
colcon build --packages-select dynamixel_test_ui

# Source the setup files
source install/setup.bash

# Run the UI node
ros2 run dynamixel_test_ui dynamixel_ui
``` 