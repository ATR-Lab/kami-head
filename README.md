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

[Installation instructions here]

## Usage

[Usage instructions here]

## Contributing

[Contribution guidelines here]

## License

[License information here]