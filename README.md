# Coffee Buddy Robot

## Overview

This repository contains the source code for the Coffee Buddy robot, a service robot designed to interact with humans in a coffee shop environment.

## Repository Structure

- `coffee_ws/`: ROS2 workspace
  - `src/`: Source packages
    - `coffee_head/`: Head control and tracking system
    - `coffee_camera/`: Camera and perception system
    - Other packages...

## Documentation

The project documentation is built using Sphinx with ReadTheDocs theme and is available at the project's GitHub Pages site.

### Accessing Documentation

The documentation is automatically built and deployed to GitHub Pages when changes are pushed to the main branch.

### Building Documentation Locally

To build the documentation locally:

1. Install the documentation dependencies:
   ```bash
   pip install -r coffee_ws/src/coffee_head/docs/requirements.txt
   ```

2. Build the documentation:
   ```bash
   cd coffee_ws/src/coffee_head/docs
   make html
   ```

3. Open `coffee_ws/src/coffee_head/docs/build/html/index.html` in your browser to view the documentation.

## Installation

### Prerequisites

- Ubuntu 24.04 (Noble Numbat)
- Python 3.12

### 1. Install ROS2 Jazzy

Follow the official ROS2 Jazzy installation guide for Ubuntu 24.04:
[ROS2 Jazzy Installation](https://docs.ros.org/en/jazzy/Installation.html)

The recommended method is to use the Debian packages:
```bash
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-jazzy-desktop
```

### 2. Install Python Dependencies

Install the required Python packages:
```bash
pip3 install -r requirements.txt
```

### 3. Build the ROS2 Workspace

```bash
cd coffee_ws
colcon build
```

### 4. Source the Setup Files

Add the following to your `~/.bashrc` to create a convenient alias for sourcing ROS2:
```bash
echo 'alias ros-source="source /opt/ros/jazzy/setup.bash && source ~/path/to/coffee-budy/coffee_ws/install/setup.bash"' >> ~/.bashrc
source ~/.bashrc
```

Alternatively, you can source the files manually each time:
```bash
source /opt/ros/jazzy/setup.bash
source ~/path/to/coffee-budy/coffee_ws/install/setup.bash
```

### 5. Running Coffee Buddy

First, make sure you've sourced the setup files:
```bash
ros-source
```

To run the head tracking:
```bash
ros2 launch coffee_head all_nodes.launch.py
```

To run the eye visuals:
```bash
ros2 launch coffee_face coffee_eyes.launch.py
```

## Usage

[Usage instructions here]

## Contributing

[Contribution guidelines here]

## License

[License information here]