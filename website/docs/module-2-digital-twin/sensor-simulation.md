---
sidebar_position: 5
title: 'Sensor Simulation (LiDAR, Depth, IMU)'
---

# Sensor Simulation (LiDAR, Depth, IMU)

Sensor simulation is critical for robotics development, allowing algorithms to be tested with realistic sensor data before deployment on physical robots.

## LiDAR Simulation

### Gazebo LiDAR Plugin
Gazebo provides realistic LiDAR simulation through plugins:

```xml
<sensor name="lidar_sensor" type="ray">
  <pose>0 0 0.2 0 0 0</pose>
  <ray>
    <scan>
      <horizontal>
        <samples>720</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
    <ros>
      <namespace>lidar</namespace>
      <remapping>~/out:=scan</remapping>
    </ros>
    <output_type>sensor_msgs/LaserScan</output_type>
  </plugin>
</sensor>
```

### Unity LiDAR Simulation
Unity can simulate LiDAR using raycasting:

```csharp
// Example LiDAR simulation in Unity
public class LidarSimulation : MonoBehaviour
{
    public int numRays = 720;
    public float maxDistance = 30.0f;
    public float fov = 360.0f;

    void Update()
    {
        float angleStep = fov / numRays;
        for (int i = 0; i < numRays; i++)
        {
            float angle = transform.eulerAngles.y + (i * angleStep) - (fov / 2);
            Vector3 direction = Quaternion.Euler(0, angle, 0) * transform.forward;

            if (Physics.Raycast(transform.position, direction, out RaycastHit hit, maxDistance))
            {
                // Process LiDAR point
                float distance = hit.distance;
            }
        }
    }
}
```

## Depth Camera Simulation

### Gazebo Depth Camera
```xml
<sensor name="depth_camera" type="depth">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10</far>
    </clip>
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
    <alwaysOn>true</alwaysOn>
    <updateRate>30.0</updateRate>
    <cameraName>depth_camera</cameraName>
    <imageTopicName>/rgb/image_raw</imageTopicName>
    <depthImageTopicName>/depth/image_raw</depthImageTopicName>
    <pointCloudTopicName>/depth/points</pointCloudTopicName>
  </plugin>
</sensor>
```

## IMU Simulation

### Gazebo IMU Sensor
```xml
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2e-4</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2e-4</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2e-4</stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>
</sensor>
```

## Sensor Fusion

### Combining Multiple Sensors
For realistic simulation, sensors should be combined to provide comprehensive environmental awareness:

- **LiDAR + Camera**: For object detection and classification
- **IMU + Encoders**: For odometry and localization
- **Depth + IMU**: For 3D reconstruction with orientation

## Performance Considerations

- Balance sensor fidelity with simulation performance
- Use appropriate update rates for each sensor type
- Consider computational requirements for real-time simulation
- Validate simulated sensor data against real sensor data