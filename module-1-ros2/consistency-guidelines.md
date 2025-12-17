# Consistency Guidelines for ROS 2 Educational Content

## Tone and Style
- Use engineering-grade, instructional tone
- No marketing language, fluff, or motivational filler
- Be precise and technical while remaining accessible
- Write for advanced Computer Science students, Robotics Engineers, and AI Engineers

## Terminology Consistency
- Use official ROS 2 terminology from ROS 2 documentation
- Always refer to "ROS 2" not just "ROS" when referring to ROS 2 specifically
- Use "nodes", "topics", "services", "actions" with lowercase
- Use "rclpy" for Python client library, "rclcpp" for C++ client library
- Use "DDS" in uppercase for Data Distribution Service
- Use "QoS" in uppercase for Quality of Service

## Technical Accuracy
- Ensure all Python code snippets are syntactically correct
- Use correct ROS 2 message types (e.g., std_msgs, sensor_msgs, geometry_msgs)
- Follow ROS 2 naming conventions (snake_case for topics, packages)
- Use appropriate QoS profiles for different data types

## Humanoid Context
- Always use humanoid robotics examples when possible
- Reference specific humanoid robot components (joints, sensors, actuators)
- Use realistic scenarios for humanoid robot applications
- Connect concepts to humanoid robot control and behavior

## Formatting Consistency
- Use consistent heading hierarchy (H1 for chapter title, H2 for sections, H3 for subsections)
- Use the same front-matter format for all chapters
- Maintain consistent code example formatting
- Use consistent bullet point and numbered list styles

## Cross-Chapter Consistency
- Use consistent terminology across all chapters
- Maintain the same level of technical detail
- Build concepts progressively from basic to advanced
- Reference previous chapters when building on prior concepts
- Prepare readers for concepts that will be covered in later chapters