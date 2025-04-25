# ROS2 Python Coding Conventions for Coffee-Buddy

This document outlines the coding conventions and best practices for ROS2 Python development in the Coffee-Buddy project.

## Code Organization

### Package Structure

```
coffee_ws/src/my_package/
├── my_package/                  # Python package
│   ├── __init__.py
│   ├── my_node.py               # Main node implementation
│   ├── utils/                   # Utility modules
│   │   ├── __init__.py
│   │   └── helpers.py
│   └── components/              # Node components
│       ├── __init__.py
│       └── component.py
├── launch/                      # Launch files
│   └── my_node.launch.py
├── config/                      # Configuration files
│   └── params.yaml
├── test/                        # Test files
│   └── test_my_node.py
├── package.xml                  # Package manifest
├── setup.py                     # Package setup
├── setup.cfg                    # Package configuration
└── README.md                    # Documentation
```

### Module Organization

For complex nodes, split functionality into separate modules with clear responsibilities:

1. **Main Node**: Handles ROS2 interface (parameters, topics, services)
2. **Components**: Implements specific functionality
3. **Utilities**: Provides shared functionality

## Coding Style

### General Guidelines

* Follow [PEP 8](https://www.python.org/dev/peps/pep-0008/) guidelines
* Use 4 spaces for indentation (not tabs)
* Limit lines to 100 characters
* Use meaningful variable and function names
* Avoid single-letter variable names except for simple loops

### Naming Conventions

* **Classes**: `CamelCase`
* **Functions/Methods**: `snake_case`
* **Variables**: `snake_case`
* **Constants**: `UPPER_SNAKE_CASE`
* **ROS2 Node Classes**: `MyNode` (CamelCase with "Node" suffix)
* **ROS2 Parameter Names**: `snake_case`
* **ROS2 Topic/Service Names**: `/snake_case_with_namespaces`

### Documentation

* Use Google-style docstrings for all classes and methods
* Add module-level docstrings explaining the purpose of the module
* Document parameters, return values, and exceptions

Example:
```python
def calculate_distance(point_a, point_b):
    """
    Calculate Euclidean distance between two points.
    
    Args:
        point_a (tuple): (x, y, z) coordinates of point A
        point_b (tuple): (x, y, z) coordinates of point B
        
    Returns:
        float: Euclidean distance between points
        
    Raises:
        ValueError: If points don't have the same dimensions
    """
```

## ROS2 Specific Guidelines

### Node Development

* Create a class inheriting from `rclpy.node.Node`
* Separate parameter handling from core functionality
* Use descriptive node and topic/service names
* Group related parameters with parameter namespace prefixes

### Parameter Management

* Declare all parameters with appropriate descriptions and types
* Use `ParameterDescriptor` to specify type and description
* Separate parameter declaration from the rest of the code
* Validate parameters after loading

Example:
```python
def _declare_parameters(self):
    """Declare all node parameters."""
    self.declare_parameter(
        'timeout', 
        5.0, 
        ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Timeout in seconds'
        )
    )
```

### Threading

* Use daemon threads for background operations
* Ensure proper synchronization for shared resources
* Always provide a way to gracefully stop threads
* Clean up resources in `destroy_node()`

### Logging

* Use the ROS2 logger instead of `print()` statements
* Use appropriate log levels: debug, info, warning, error, fatal
* Include relevant context in log messages
* Use structured logging for complex information

Example:
```python
self.get_logger().info(f"Processing data: size={data_size}, mode={mode}")
```

## Error Handling

* Use try-except blocks around potential failure points
* Log detailed error information
* Clean up resources in case of errors
* Provide clear error messages and context

Example:
```python
try:
    result = self._process_data(data)
    return result
except Exception as e:
    self.get_logger().error(f"Error processing data: {str(e)}")
    # Clean up resources
    return None
```

## Resource Management

* Clean up resources in `destroy_node()`
* Join threads with timeout
* Use context managers when appropriate
* Check for resource availability before use

## Testing

* Write unit tests for non-ROS functionality
* Write integration tests for ROS components
* Use mock objects for external dependencies
* Test edge cases and error conditions

## Version Control

* Write clear, concise commit messages
* Reference issue numbers in commit messages
* Use feature branches for development
* Review code before merging

By following these conventions, we can ensure consistent, maintainable code across the Coffee-Buddy project. 