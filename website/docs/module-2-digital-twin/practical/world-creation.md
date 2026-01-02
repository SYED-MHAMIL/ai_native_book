---
sidebar_position: 1
title: "Practical World Creation and Environment Setup"
description: "Hands-on guide to creating and testing simulation environments for humanoid robots"
tags: [gazebo, simulation, practical, world-creation]
---

# Practical World Creation and Environment Setup

## Overview

This practical guide will walk you through creating and testing simulation environments for humanoid robots using Gazebo. You'll build actual worlds, place objects, configure physics, and test your humanoid model in various scenarios.

## Prerequisites

Before starting this practical session, ensure you have:

- Gazebo Garden or Citadel installed and verified
- ROS 2 Humble with Gazebo ROS packages installed
- Basic understanding of SDF/XML format
- Access to a humanoid robot model (or the ability to create one)

## Setting Up Your Simulation Environment

### Installing Required Packages

```bash
# Install Gazebo ROS packages
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros
sudo apt install ros-humble-gazebo-dev

# Install additional tools
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-robot-state-publisher
```

### Creating a Simulation Package

```bash
cd ~/ros2_humanoid_ws/src
ros2 pkg create --build-type ament_cmake humanoid_simulation --dependencies rclcpp gazebo_ros_pkgs
```

## Creating Your First World

### Basic World File

Create `humanoid_simulation/worlds/basic_humanoid_world.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="basic_humanoid_world">
    <!-- Physics engine configuration -->
    <physics name="ode_physics" default="0" type="ode">
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

    <!-- Include standard models -->
    <include>
      <uri>model://sun</uri>
    </include>

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Add a simple humanoid model placeholder -->
    <model name="simple_humanoid">
      <pose>0 0 1 0 0 0</pose>
      <link name="base_link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.1</iyy>
            <iyz>0</iyz>
            <izz>0.1</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.3 0.8</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
            <diffuse>0.8 0.2 0.2 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.3 0.8</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>

    <!-- Add some obstacles for testing -->
    <model name="test_obstacle_1">
      <pose>2 1 0.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 1.0</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 1.0</size>
            </box>
          </geometry>
          <material>
            <ambient>0.2 0.8 0.2 1</ambient>
            <diffuse>0.2 0.8 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="test_obstacle_2">
      <pose>-2 -1 0.3 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.3</radius>
              <length>0.6</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.3</radius>
              <length>0.6</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.2 0.2 0.8 1</ambient>
            <diffuse>0.2 0.2 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### Launching the World

Create a launch file to start your world with ROS 2. Create `humanoid_simulation/launch/basic_world.launch.py`:

```python
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Path to the world file
    world_path = PathJoinSubstitution([
        FindPackageShare('humanoid_simulation'),
        'worlds',
        'basic_humanoid_world.sdf'
    ])

    # Launch Gazebo with the world
    gz_sim = ExecuteProcess(
        cmd=[
            'gz', 'sim', '-r',
            world_path
        ],
        output='screen'
    )

    return LaunchDescription([
        gz_sim
    ])
```

## Creating a More Complex Environment

### Indoor Environment World

Create `humanoid_simulation/worlds/indoor_humanoid_world.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="indoor_humanoid_world">
    <!-- Physics engine configuration -->
    <physics name="ode_physics" default="0" type="ode">
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

    <!-- Custom lighting -->
    <light name="indoor_light_1" type="point">
      <pose>0 0 3 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>10</range>
        <linear>0.1</linear>
        <quadratic>0.01</quadratic>
      </attenuation>
    </light>

    <!-- Indoor environment -->
    <model name="room_floor">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>8 6 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>8 6 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Walls -->
    <model name="wall_north">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>8 0.2 3</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>8 0.2 3</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="wall_south">
      <pose>0 -6 1.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>8 0.2 3</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>8 0.2 3</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="wall_east">
      <pose>4 0 1.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.2 6 3</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.2 6 3</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="wall_west">
      <pose>-4 0 1.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.2 6 3</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.2 6 3</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Furniture -->
    <model name="table">
      <pose>0 2 0.5 0 0 0</pose>
      <static>true</static>
      <link name="top">
        <collision name="collision">
          <geometry>
            <box>
              <size>1.5 0.8 0.05</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1.5 0.8 0.05</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.6 0.2 1</ambient>
            <diffuse>0.8 0.6 0.2 1</diffuse>
          </material>
        </visual>
      </link>
      <link name="leg1">
        <pose>-0.6 -0.3 0.25 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 0.1 0.5</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 0.1 0.5</size>
            </box>
          </geometry>
          <material>
            <ambient>0.4 0.3 0.1 1</ambient>
            <diffuse>0.4 0.3 0.1 1</diffuse>
          </material>
        </visual>
      </link>
      <link name="leg2">
        <pose>0.6 -0.3 0.25 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 0.1 0.5</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 0.1 0.5</size>
            </box>
          </geometry>
          <material>
            <ambient>0.4 0.3 0.1 1</ambient>
            <diffuse>0.4 0.3 0.1 1</diffuse>
          </material>
        </visual>
      </link>
      <link name="leg3">
        <pose>-0.6 0.3 0.25 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 0.1 0.5</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 0.1 0.5</size>
            </box>
          </geometry>
          <material>
            <ambient>0.4 0.3 0.1 1</ambient>
            <diffuse>0.4 0.3 0.1 1</diffuse>
          </material>
        </visual>
      </link>
      <link name="leg4">
        <pose>0.6 0.3 0.25 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 0.1 0.5</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 0.1 0.5</size>
            </box>
          </geometry>
          <material>
            <ambient>0.4 0.3 0.1 1</ambient>
            <diffuse>0.4 0.3 0.1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Add a humanoid placeholder -->
    <model name="indoor_humanoid">
      <pose>-2 0 1 0 0 0</pose>
      <link name="base_link">
        <inertial>
          <mass>50.0</mass>
          <inertia>
            <ixx>1.0</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1.0</iyy>
            <iyz>0</iyz>
            <izz>1.0</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.3 0.2 1.7</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.8 1</ambient>
            <diffuse>0.8 0.2 0.8 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.3 0.2 1.7</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

## Creating a Launch File for the Indoor World

Create `humanoid_simulation/launch/indoor_world.launch.py`:

```python
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Path to the indoor world file
    world_path = PathJoinSubstitution([
        FindPackageShare('humanoid_simulation'),
        'worlds',
        'indoor_humanoid_world.sdf'
    ])

    # Launch Gazebo with the indoor world
    gz_sim = ExecuteProcess(
        cmd=[
            'gz', 'sim', '-r',
            world_path
        ],
        output='screen'
    )

    return LaunchDescription([
        gz_sim
    ])
```

## Creating a Complex Terrain World

Create `humanoid_simulation/worlds/terrain_world.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="terrain_humanoid_world">
    <!-- Physics engine configuration -->
    <physics name="ode_physics" default="0" type="ode">
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

    <!-- Include standard models -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Complex terrain using heightmap -->
    <model name="terrain">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <heightmap>
              <uri>file://media/materials/textures/terrain_difficult.png</uri>
              <size>20 20 3</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <heightmap>
              <uri>file://media/materials/textures/terrain_difficult.png</uri>
              <size>20 20 3</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
        </visual>
      </link>
    </model>

    <!-- Add some obstacles -->
    <model name="obstacle_1">
      <pose>5 3 1 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
            <diffuse>0.8 0.2 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="ramp">
      <pose>-3 0 0.5 0 0 0.5</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>3 1 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>3 1 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Humanoid model -->
    <model name="terrain_humanoid">
      <pose>0 0 2 0 0 0</pose>
      <link name="base_link">
        <inertial>
          <mass>50.0</mass>
          <inertia>
            <ixx>1.0</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1.0</iyy>
            <iyz>0</iyz>
            <izz>1.0</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.3 0.2 1.7</size>
            </box>
          </geometry>
          <material>
            <ambient>0.2 0.8 0.8 1</ambient>
            <diffuse>0.2 0.8 0.8 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.3 0.2 1.7</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

## Testing Your Worlds

### Building the Package

```bash
cd ~/ros2_humanoid_ws
colcon build --packages-select humanoid_simulation
source install/setup.bash
```

### Running the Basic World

```bash
# Launch the basic world
ros2 launch humanoid_simulation basic_world.launch.py

# In another terminal, check the Gazebo topics
gz topic -l
```

### Running the Indoor World

```bash
# Launch the indoor world
ros2 launch humanoid_simulation indoor_world.launch.py
```

## Creating a Robot Integration Test

Create `humanoid_simulation/launch/humanoid_with_gazebo.launch.py`:

```python
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch.actions import TimerAction

def generate_launch_description():
    # Path to the world file
    world_path = PathJoinSubstitution([
        FindPackageShare('humanoid_simulation'),
        'worlds',
        'basic_humanoid_world.sdf'
    ])

    # Launch Gazebo with the world
    gz_sim = ExecuteProcess(
        cmd=[
            'gz', 'sim', '-r',
            world_path
        ],
        output='screen'
    )

    # Optional: Launch RViz for visualization
    rviz_config = PathJoinSubstitution([
        FindPackageShare('humanoid_simulation'),
        'config',
        'simulation.rviz'
    ])

    # Create a simple RViz config file if it doesn't exist
    # This is just an example - you can create a more complex one
    rviz_cmd = ExecuteProcess(
        cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', rviz_config],
        output='screen',
        condition=launch.conditions.IfCondition(
            launch.substitutions.LaunchConfiguration('use_rviz', default='false')
        )
    )

    return LaunchDescription([
        gz_sim,
        rviz_cmd
    ])
```

## Advanced Environment: Multi-Robot Testing

Create `humanoid_simulation/worlds/multi_robot_world.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="multi_robot_world">
    <!-- Physics engine configuration -->
    <physics name="ode_physics" default="0" type="ode">
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

    <!-- Include standard models -->
    <include>
      <uri>model://sun</uri>
    </include>

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Robot 1 -->
    <model name="humanoid_1">
      <pose>-2 0 1 0 0 0</pose>
      <link name="base_link">
        <inertial>
          <mass>50.0</mass>
          <inertia>
            <ixx>1.0</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1.0</iyy>
            <iyz>0</iyz>
            <izz>1.0</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.3 0.2 1.7</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
            <diffuse>0.8 0.2 0.2 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.3 0.2 1.7</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>

    <!-- Robot 2 -->
    <model name="humanoid_2">
      <pose>2 0 1 0 0 0</pose>
      <link name="base_link">
        <inertial>
          <mass>50.0</mass>
          <inertia>
            <ixx>1.0</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1.0</iyy>
            <iyz>0</iyz>
            <izz>1.0</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.3 0.2 1.7</size>
            </box>
          </geometry>
          <material>
            <ambient>0.2 0.8 0.2 1</ambient>
            <diffuse>0.2 0.8 0.2 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.3 0.2 1.7</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>

    <!-- Obstacles for coordination testing -->
    <model name="narrow_passage">
      <pose>0 3 0.5 0 0 0</pose>
      <static>true</static>
      <link name="left_pillar">
        <pose>-0.5 0 0.5 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.2 0.2 1.0</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.2 0.2 1.0</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
      <link name="right_pillar">
        <pose>0.5 0 0.5 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.2 0.2 1.0</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.2 0.2 1.0</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## Environment Validation Script

Create a Python script to validate your environments:

Create `humanoid_simulation/scripts/validate_environment.py`:

```python
#!/usr/bin/env python3

import subprocess
import time
import sys
import os

def validate_gazebo_installation():
    """Check if Gazebo is properly installed"""
    try:
        result = subprocess.run(['gz', '--version'],
                              capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            print(f"✓ Gazebo installed: {result.stdout.strip()}")
            return True
        else:
            print(f"✗ Gazebo not working: {result.stderr}")
            return False
    except FileNotFoundError:
        print("✗ Gazebo not installed")
        return False
    except subprocess.TimeoutExpired:
        print("✗ Gazebo command timed out")
        return False

def validate_world_file(world_path):
    """Check if a world file is valid SDF"""
    try:
        result = subprocess.run(['gz', 'sdf', '-k', world_path],
                              capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            print(f"✓ World file valid: {world_path}")
            return True
        else:
            print(f"✗ World file invalid: {world_path}")
            print(f"  Error: {result.stderr}")
            return False
    except subprocess.TimeoutExpired:
        print(f"✗ World validation timed out: {world_path}")
        return False

def test_gazebo_launch(world_path):
    """Test launching a world (non-blocking)"""
    try:
        # Launch Gazebo with the world in headless mode
        process = subprocess.Popen([
            'gz', 'sim', '-s',  # headless mode
            world_path
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        # Wait a bit to see if it starts successfully
        time.sleep(2)

        # Check if process is still running (good sign)
        if process.poll() is None:
            print(f"✓ World launch successful: {world_path}")
            process.terminate()  # Stop the process
            return True
        else:
            stdout, stderr = process.communicate()
            print(f"✗ World launch failed: {world_path}")
            print(f"  Error: {stderr.decode()}")
            return False
    except Exception as e:
        print(f"✗ World launch error: {world_path} - {str(e)}")
        return False

def main():
    print("=== Environment Validation Script ===")

    # Validate Gazebo installation
    if not validate_gazebo_installation():
        print("Gazebo installation failed. Please install Gazebo before continuing.")
        sys.exit(1)

    # Find world files in the package
    package_dir = os.path.join(os.path.expanduser('~'), 'ros2_humanoid_ws', 'src', 'humanoid_simulation')
    worlds_dir = os.path.join(package_dir, 'worlds')

    if not os.path.exists(worlds_dir):
        print(f"Worlds directory not found: {worlds_dir}")
        sys.exit(1)

    world_files = [f for f in os.listdir(worlds_dir) if f.endswith('.sdf')]

    if not world_files:
        print(f"No SDF world files found in {worlds_dir}")
        sys.exit(1)

    print(f"Found {len(world_files)} world files to validate:")

    all_valid = True
    for world_file in world_files:
        world_path = os.path.join(worlds_dir, world_file)

        print(f"\nValidating: {world_file}")

        # Validate SDF syntax
        if not validate_world_file(world_path):
            all_valid = False
            continue

        # Test launch (optional, can be skipped for faster validation)
        print("  Skipping launch test (set LAUNCH_TEST=1 to enable)")
        # if 'LAUNCH_TEST' in os.environ:
        #     if not test_gazebo_launch(world_path):
        #         all_valid = False

    print(f"\n=== Validation Complete ===")
    if all_valid:
        print("✓ All environments validated successfully!")
        print("\nNext steps:")
        print("1. Build your ROS 2 package: colcon build --packages-select humanoid_simulation")
        print("2. Source the environment: source install/setup.bash")
        print("3. Launch a world: ros2 launch humanoid_simulation basic_world.launch.py")
    else:
        print("✗ Some environments failed validation.")
        sys.exit(1)

if __name__ == '__main__':
    main()
```

## Running Your Validation

Make the script executable and run it:

```bash
chmod +x ~/ros2_humanoid_ws/src/humanoid_simulation/scripts/validate_environment.py
cd ~/ros2_humanoid_ws
colcon build --packages-select humanoid_simulation
source install/setup.bash
python3 src/humanoid_simulation/scripts/validate_environment.py
```

## Module Summary

In this practical session, you:

- Created multiple simulation environments (basic, indoor, terrain, multi-robot)
- Set up ROS 2 launch files for each environment
- Validated your environments using a Python script
- Learned advanced SDF techniques for complex environments
- Created realistic testing scenarios for humanoid robots

These practical skills will help you create effective simulation environments for testing and validating your humanoid robots.

## Next Steps

Continue to the [Advanced Topics](../advanced/index.md) section to learn about more sophisticated simulation techniques, including sensor integration, physics tuning, and transfer learning strategies.