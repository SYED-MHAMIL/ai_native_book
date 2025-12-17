# Code Example Formatting Standards

## Python Code Snippets for ROS 2

### General Guidelines
- Use Python 3.8+ syntax compatible with ROS 2 Humble
- Include proper imports at the top
- Use meaningful variable names
- Add comments explaining non-obvious code
- Follow PEP 8 style guidelines

### Code Block Format
```python
# Brief description of what this code does
import rclpy
from rclpy.node import Node
# ... additional imports as needed

class ExampleNode(Node):
    def __init__(self):
        super().__init__('example_node')
        # Initialization code here

    def example_method(self):
        # Method implementation
        pass

def main(args=None):
    rclpy.init(args=args)
    node = ExampleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Specific Requirements for ROS 2 Examples
- Always inherit from `rclpy.node.Node`
- Use proper logger: `self.get_logger().info("message")`
- Implement proper cleanup in `destroy_node()`
- Handle shutdown gracefully with try/except blocks
- Use appropriate QoS profiles when needed

### Humanoid Context Examples
- Use humanoid-specific examples (joint controllers, IMU sensors, etc.)
- Include realistic values and parameters for humanoid robots
- Show practical applications rather than abstract examples