---
sidebar_position: 2
title: "World Creation and Environment Modeling"
description: "Creating realistic simulation environments for humanoid robot testing and development"
tags: [gazebo, simulation, world-modeling, environment]
---

# World Creation and Environment Modeling

## Overview

Creating realistic simulation environments is crucial for developing and testing humanoid robots. Well-designed worlds allow for safe testing of control algorithms, sensor systems, and AI behaviors before deployment on physical hardware.

## Gazebo World Structure

Gazebo worlds are defined using SDF (Simulation Description Format) files that specify the environment, physics properties, lighting, and initial conditions.

### Basic World Template

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="humanoid_test_world">
    <!-- Physics engine configuration -->
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Include standard models -->
    <include>
      <uri>model://sun</uri>
    </include>

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Custom environment elements -->
    <!-- Add obstacles, furniture, etc. here -->
  </world>
</sdf>
```

## Terrain and Ground Types

### Flat Ground Plane
The most basic environment for testing locomotion and basic behaviors:

```xml
<model name="ground_plane">
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <plane>
          <normal>0 0 1</normal>
          <size>100 100</size>
        </plane>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <plane>
          <normal>0 0 1</normal>
          <size>100 100</size>
        </plane>
      </geometry>
      <material>
        <ambient>0.7 0.7 0.7 1</ambient>
        <diffuse>0.7 0.7 0.7 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

### Complex Terrain
For more realistic testing with uneven surfaces:

```xml
<model name="complex_terrain">
  <static>true</static>
  <link name="terrain_link">
    <collision name="collision">
      <geometry>
        <heightmap>
          <uri>model://my_terrain/heightmap.png</uri>
          <size>10 10 2</size>
        </heightmap>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <heightmap>
          <uri>model://my_terrain/heightmap.png</uri>
          <size>10 10 2</size>
        </heightmap>
      </geometry>
    </visual>
  </link>
</model>
```

## Obstacle and Object Placement

### Static Obstacles
Creating obstacles for navigation and path planning tests:

```xml
<model name="obstacle_box">
  <pose>2 0 0.5 0 0 0</pose>
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
      <material>
        <ambient>0.8 0.2 0.2 1</ambient>
        <diffuse>0.8 0.2 0.2 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

### Interactive Objects
Objects that can be manipulated by the humanoid robot:

```xml
<model name="movable_box">
  <pose>3 1 0.5 0 0 0</pose>
  <link name="link">
    <inertial>
      <mass>2.0</mass>
      <inertia>
        <ixx>0.1667</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>0.1667</iyy>
        <iyz>0</iyz>
        <izz>0.1667</izz>
      </inertia>
    </inertial>
    <collision name="collision">
      <geometry>
        <box>
          <size>0.3 0.3 0.3</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>0.3 0.3 0.3</size>
        </box>
      </geometry>
      <material>
        <ambient>0.2 0.2 0.8 1</ambient>
        <diffuse>0.2 0.2 0.8 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

## Sensor Testing Environments

### Calibration Rooms
Simple, well-defined environments for sensor calibration:

```xml
<model name="calibration_target">
  <pose>0 0 1 0 0 0</pose>
  <static>true</static>
  <!-- Checkerboard pattern for camera calibration -->
  <link name="link">
    <visual name="visual">
      <geometry>
        <box>
          <size>2 2 0.01</size>
        </box>
      </geometry>
      <!-- Add checkerboard material -->
    </visual>
  </link>
</model>
```

### Dynamic Environments
Environments with moving elements for advanced testing:

```xml
<model name="moving_obstacle">
  <link name="link">
    <inertial>
      <mass>1.0</mass>
      <inertia>
        <ixx>0.4167</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>0.4167</iyy>
        <iyz>0</iyz>
        <izz>0.4167</izz>
      </inertia>
    </inertial>
    <collision name="collision">
      <geometry>
        <sphere>
          <radius>0.3</radius>
        </sphere>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <sphere>
          <radius>0.3</radius>
        </sphere>
      </geometry>
      <material>
        <ambient>0.8 0.6 0.2 1</ambient>
        <diffuse>0.8 0.6 0.2 1</diffuse>
      </material>
    </visual>
  </link>
  <!-- Add joint for movement -->
  <joint name="x_axis" type="prismatic">
    <parent>world</parent>
    <child>link</child>
    <axis>
      <xyz>1 0 0</xyz>
    </axis>
  </joint>
</model>
```

## Physics Configuration

### Realistic Physics Settings
For accurate simulation of humanoid robot dynamics:

```xml
<physics name="realistic_physics" default="0" type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  <gravity>0 0 -9.8</gravity>
  <ode>
    <solver>
      <type>quick</type>
      <iters>10</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

## Lighting and Visual Effects

### Indoor Lighting
For simulating indoor environments with artificial lighting:

```xml
<light name="room_light" type="point">
  <pose>0 0 3 0 0 0</pose>
  <diffuse>0.8 0.8 0.8 1</diffuse>
  <specular>0.2 0.2 0.2 1</specular>
  <attenuation>
    <range>10</range>
    <linear>0.1</linear>
    <quadratic>0.01</quadratic>
  </attenuation>
</light>
```

## World Best Practices

### Performance Optimization
- Use simplified collision geometries for complex visual models
- Limit the number of dynamic objects for real-time performance
- Use static models where possible
- Optimize physics parameters for your specific use case

### Realism vs. Performance
- Balance visual fidelity with simulation speed
- Use domain randomization to improve robustness
- Validate simulation results against analytical models when possible

## Integration with ROS 2

World files can be launched directly with ROS 2 launch files:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    world_path = PathJoinSubstitution([
        FindPackageShare("my_robot_gazebo"),
        "worlds",
        "humanoid_test_world.sdf"
    ])

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py"
                ])
            ]),
            launch_arguments={"world": world_path}.items()
        )
    ])
```

## Module Summary

This section covered the fundamentals of creating simulation environments:

- World structure and SDF format
- Terrain and ground types
- Obstacle and object placement
- Physics configuration
- Lighting and visual effects
- Best practices for performance and realism

Well-designed simulation environments are essential for safe and efficient robot development.

## Next Steps

Continue to the [Practical Implementation](../practical/world-creation.md) section to learn how to create and test these environments with your humanoid robot.