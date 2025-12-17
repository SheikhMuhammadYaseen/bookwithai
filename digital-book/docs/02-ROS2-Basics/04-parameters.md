---
id: ros2-parameters
title: ROS2 Parameters
sidebar_label: Parameters
sidebar_position: 4
---

# ROS2 Parameters: Essential Guide

## Understanding ROS2 Parameters {#ch2-sec4-understanding-ros2-parameters}

**Parameters** are runtime configuration values for nodes. They allow you to tune node behavior without changing code. Think of them as settings or preferences for your ROS2 nodes.

**For complete parameter system theory**: See ROS2 parameters documentation.

## Key Characteristics {#ch2-sec4-key-characteristics-parameters}

- **Runtime Configuration**: Change values without restarting
- **Dynamic Updates**: Modify parameters while node runs  
- **Multiple Types**: Support integers, floats, strings, booleans, arrays
- **Namespaced**: Parameters belong to specific nodes

## Basic Parameter Usage {#ch2-sec4-basic-usage}

### Declaring and Using Parameters
```python
import rclpy
from rclpy.node import Node

class ParamNode(Node):
    def __init__(self):
        super().__init__('param_node')
        
        # Declare parameters with defaults
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('name', 'robot')
        self.declare_parameter('enabled', True)
        
        # Get parameter values
        speed = self.get_parameter('speed').value
        name = self.get_parameter('name').value
        enabled = self.get_parameter('enabled').value
        
        self.get_logger().info(f'Config: {name}, Speed: {speed}, Enabled: {enabled}')
```

### Parameter Change Callback
```python
class ReactiveParamNode(Node):
    def __init__(self):
        super().__init__('reactive_node')
        
        self.declare_parameter('threshold', 0.5)
        
        # Add callback for parameter changes
        self.add_on_set_parameters_callback(self.param_change_callback)
    
    def param_change_callback(self, params):
        for param in params:
            self.get_logger().info(f'{param.name} changed to {param.value}')
        return rclpy.node.SetParametersResult(successful=True)
```

**For advanced parameter patterns**: Check ROS2 dynamic reconfigure tutorials.

## Setting Parameters {#ch2-sec4-setting-parameters}

### 1. Command Line
```bash
# Set when starting node
ros2 run your_package param_node --ros-args -p speed:=2.0 -p name:="rover"

# Change while node runs
ros2 param set /param_node speed 3.0
ros2 param set /param_node enabled false
```

### 2. Launch File
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package',
            executable='param_node',
            parameters=[
                {'speed': 2.5},
                {'name': 'explorer'},
                {'threshold': 0.7}
            ]
        ),
    ])
```

### 3. YAML Config File
```yaml
param_node:
  ros__parameters:
    speed: 1.5
    name: "robot1"
    enabled: true
    thresholds: [0.1, 0.5, 0.9]
```

**Usage**:
```python
# In launch file
Node(
    package='your_package',
    executable='param_node',
    parameters=['config/params.yaml']
)
```

**For YAML parameter files**: See ROS2 configuration guides.

## Parameter Commands {#ch2-sec4-parameter-commands}

| Command | Description | Example |
|---------|-------------|---------|
| `ros2 param list` | List node parameters | `ros2 param list /node_name` |
| `ros2 param get` | Get parameter value | `ros2 param get /node_name speed` |
| `ros2 param set` | Set parameter value | `ros2 param set /node_name speed 2.0` |
| `ros2 param dump` | Save parameters to YAML | `ros2 param dump /node_name > params.yaml` |
| `ros2 param load` | Load parameters from YAML | `ros2 param load /node_name params.yaml` |

**Example workflow**:
```bash
# List all parameters of a node
ros2 param list /param_node

# Get current value
ros2 param get /param_node speed

# Change value
ros2 param set /param_node speed 3.5

# Save all parameters to file
ros2 param dump /param_node > my_params.yaml
```

## Parameter Types {#ch2-sec4-parameter-types}

### Supported Types
```python
# Different parameter types
self.declare_parameter('count', 10)          # Integer
self.declare_parameter('speed', 1.5)         # Double (float)
self.declare_parameter('name', 'robot')      # String
self.declare_parameter('enabled', True)      # Boolean
self.declare_parameter('gains', [0.1, 0.2, 0.3])  # Array
self.declare_parameter('config', {'key': 'value'})  # Dictionary
```

### Type Checking and Validation
```python
def param_change_callback(self, params):
    """Validate parameter changes"""
    for param in params:
        if param.name == 'speed':
            # Validate speed is positive
            if param.value <= 0:
                self.get_logger().error('Speed must be positive')
                return rclpy.node.SetParametersResult(successful=False)
        
        elif param.name == 'threshold':
            # Validate range
            if not 0.0 <= param.value <= 1.0:
                self.get_logger().error('Threshold must be 0-1')
                return rclpy.node.SetParametersResult(successful=False)
    
    return rclpy.node.SetParametersResult(successful=True)
```

## Real-World Examples {#ch2-sec4-real-world-examples}

### 1. PID Controller Tuning
```python
class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        
        # Tunable parameters
        self.declare_parameter('kp', 1.0)  # Proportional gain
        self.declare_parameter('ki', 0.1)  # Integral gain  
        self.declare_parameter('kd', 0.01) # Derivative gain
        self.declare_parameter('max_output', 100.0)
        
        # React to parameter changes
        self.add_on_set_parameters_callback(self.update_pid_params)
    
    def update_pid_params(self, params):
        # Reconfigure PID when gains change
        self.get_logger().info('PID parameters updated')
        return rclpy.node.SetParametersResult(successful=True)
```

### 2. Sensor Configuration
```python
class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        
        # Configurable sensor settings
        self.declare_parameter('rate', 10.0)      # Hz
        self.declare_parameter('range', 100.0)    # meters
        self.declare_parameter('calibration', 1.0) # factor
        self.declare_parameter('filter_enabled', True)
```

**For more practical examples**: See ROS2 demo packages.

## Best Practices {#ch2-sec4-best-practices}

### 1. **Set Sensible Defaults**
```python
# Always provide reasonable defaults
self.declare_parameter('timeout', 5.0)    # 5 seconds default
self.declare_parameter('max_retries', 3)  # 3 retries default
```

### 2. **Validate Parameter Changes**
```python
# Prevent invalid configurations
def validate_param_change(self, param):
    if param.name == 'speed' and param.value < 0:
        return False  # Reject negative speed
    return True
```

### 3. **Document Parameters**
```python
# Document expected ranges/values in comments
self.declare_parameter('threshold', 0.5)  # Range: 0.0-1.0
self.declare_parameter('mode', 'normal')  # Options: 'normal', 'fast', 'safe'
```

### 4. **Group Related Parameters**
```yaml
# In YAML config
sensor_params:
  rate: 30
  resolution: "1080p"
  exposure: 0.01

control_params:
  kp: 1.5
  ki: 0.2
  kd: 0.05
```

**For parameter organization**: Read ROS2 configuration management guides.

## Common Issues {#ch2-sec4-common-issues}

1. **Parameter Not Found**: Ensure parameter is declared before use
2. **Type Mismatch**: Check parameter types match declarations
3. **Callback Not Firing**: Verify callback is properly registered
4. **Namespace Issues**: Use full node name when accessing parameters

**Troubleshooting tips**: Available in ROS2 parameter documentation.

## When to Use Parameters {#ch2-sec4-when-to-use}

### ✅ **Use Parameters For:**
- Configuration settings
- Tuning values (PID gains, thresholds)
- Operational modes
- Feature toggles
- Calibration values

### ❌ **Avoid Parameters For:**
- High-frequency data (use topics)
- Command/response (use services)
- Large data structures
- Real-time control signals

## Quick Reference {#ch2-sec4-quick-reference}

```python
# Declaration
self.declare_parameter(name, default_value)

# Reading  
value = self.get_parameter(name).value

# Setting from code
self.set_parameters([Parameter(name, value)])

# Callback for changes
self.add_on_set_parameters_callback(callback_function)
```

## Key Takeaways {#ch2-sec4-key-takeaways}

1. **Parameters = Runtime configuration**
2. **Use for tunable values and settings**
3. **Always provide sensible defaults**
4. **Validate parameter changes**
5. **Use CLI tools for testing and debugging**

**Next**: For complex configurations, explore ROS2 component composition.
