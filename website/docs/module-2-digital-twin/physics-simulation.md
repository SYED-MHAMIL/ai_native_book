---
sidebar_position: 2
title: 'Physics & Collision Simulation'
---

# Physics & Collision Simulation

Physics simulation is the foundation of digital twins, enabling realistic modeling of robot interactions with the environment. In this section, we'll explore physics simulation in Gazebo.

## Physics Engines

Gazebo supports multiple physics engines:
- **ODE (Open Dynamics Engine)**: Default engine, good for general-purpose simulation
- **Bullet**: Fast and robust for collision detection
- **DART**: Advanced dynamics with support for soft-body simulation

## Collision Detection

Collision detection is essential for:
- Preventing objects from passing through each other
- Computing contact forces
- Triggering events based on contact

### Collision Properties
```xml
<collision name="collision">
  <geometry>
    <box>
      <size>1 1 1</size>
    </box>
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>
        <mu2>1.0</mu2>
      </ode>
    </friction>
    <contact>
      <ode>
        <kp>1e+16</kp>
        <kd>1e+13</kd>
      </ode>
    </contact>
  </surface>
</collision>
```

## Physics Parameters

### Gravity
Gravity is typically set to Earth's gravity (9.81 m/s²) but can be modified:
```xml
<world>
  <gravity>0 0 -9.8</gravity>
</world>
```

### Damping
Damping helps stabilize simulations by reducing oscillations:
```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
</physics>
```

## Best Practices

- Use appropriate collision shapes for performance
- Balance accuracy with computational cost
- Test with different physics engines for optimal results
- Validate simulation results against real-world data