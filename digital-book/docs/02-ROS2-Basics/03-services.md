---
id: ros2-services
title: ROS2 Services
sidebar_label: Services
sidebar_position: 3
---

# ROS2 Services: Essential Guide

## Understanding ROS2 Services {#ch2-sec3-understanding-ros2-services}

**Services** provide a synchronous request-response communication pattern in ROS2. Unlike topics (continuous data streams), services are designed for one-time operations that require immediate feedback.

**For in-depth theory**: Read the official ROS2 services documentation.

### Key Characteristics {#ch2-sec3-key-characteristics-services}

- **Synchronous**: Client waits for response
- **One-to-Many**: One server handles multiple clients
- **Typed Interface**: Defined in `.srv` files
- **Blocking/Non-blocking**: Both patterns supported

## Service Definition {#ch2-sec3-service-definition}

Service definitions use `.srv` files in a `srv/` directory.

**Example structure**:
```
# Request fields
int64 a
int64 b
---
# Response fields  
int64 sum
```

**For custom service creation**: See ROS2 interface tutorials.

## Service Server Implementation {#ch2-sec3-service-server}

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MathService(Node):
    def __init__(self):
        super().__init__('math_service')
        self.srv = self.create_service(
            AddTwoInts, 
            'add_two_ints', 
            self.add_callback
        )
    
    def add_callback(self, request, response):
        response.sum = request.a + request.b
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MathService()
    rclpy.spin(node)
    rclpy.shutdown()
```

## Service Client Implementation {#ch2-sec3-service-client}

### Synchronous Client
```python
class MathClient(Node):
    def __init__(self):
        super().__init__('math_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        
    def call_service(self, a, b):
        # Wait for service
        if not self.client.wait_for_service(timeout_sec=1.0):
            return None
        
        # Create and send request
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        return future.result().sum if future.result() else None
```

### Asynchronous Client
```python
class AsyncMathClient(Node):
    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        
        future = self.client.call_async(request)
        future.add_done_callback(lambda f: self.handle_response(f))
    
    def handle_response(self, future):
        response = future.result()
        # Process response here
```

**For advanced client patterns**: Check ROS2 async programming guides.

## ROS2 Service Commands {#ch2-sec3-service-commands}

| Command | Description |
|---------|-------------|
| `ros2 service list` | List all services |
| `ros2 service call` | Call service from CLI |
| `ros2 service type` | Show service type |
| `ros2 interface show` | Show .srv definition |

**Example CLI usage**:
```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"
```


**For more examples**: See ROS2 demo packages.


## Best Practices {#ch2-sec3-best-practices}

### 1. **Response Time**
- Services should return quickly (< 100ms)
- For long operations, use Actions

### 2. **Error Handling**
```python
def service_callback(self, request, response):
    try:
        response.result = process(request)
        response.success = True
    except Exception as e:
        response.success = False
        response.error = str(e)
    return response
```

### 3. **Timeout Management**
```python
# Always use timeouts
if not client.wait_for_service(timeout_sec=2.0):
    # Handle timeout
    return None
```

**For production best practices**: Review ROS2 reliability patterns.


## Common Issues {#ch2-sec3-pitfalls}

1. **Service Not Found**: Check names and types match
2. **Blocking Forever**: Always implement timeouts  
3. **Memory Issues**: Proper cleanup required
4. **Network Problems**: Handle disconnected services

**Troubleshooting guide**: Available in ROS2 documentation.


## Services vs Topics {#ch2-sec3-comparison}

| **Services** | **Topics** |
|--------------|------------|
| Request-Response | Publish-Subscribe |
| Synchronous | Asynchronous |
| One-time operations | Continuous streams |
| Higher latency | Lower latency |

**When to use which**:
- **Services**: "Get current temperature", "Move robot to position"
- **Topics**: "Continuous sensor data", "Video stream"
