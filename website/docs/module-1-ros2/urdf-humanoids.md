---
sidebar_position: 5
title: 'URDF for Humanoid Robots'
---

# URDF for Humanoid Robots

URDF (Unified Robot Description Format) is an XML format for representing a robot model. For humanoid robots, URDF describes the physical structure, joints, and kinematic properties.

## URDF Structure

A typical humanoid URDF includes:
- Links: Rigid parts of the robot
- Joints: Connections between links
- Materials: Visual properties
- Transmissions: Actuator information

## Example URDF for Humanoid Robot

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
    </visual>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.1 0.3 0.2"/>
      </geometry>
    </visual>
  </link>

  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.25"/>
  </joint>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
  </link>

  <joint name="torso_to_head" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.25"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>
</robot>
```

## Key Considerations for Humanoids

### Kinematic Chains
Humanoid robots typically have multiple kinematic chains (arms, legs) that need to be properly defined in URDF.

### Joint Limits
Proper joint limits are crucial for realistic humanoid movement simulation.

### Mass Properties
Accurate mass and inertia properties are important for physics simulation.

## Integration with ROS 2

URDF files can be loaded into ROS 2 using the robot_state_publisher package, which publishes the robot's joint states and transforms.