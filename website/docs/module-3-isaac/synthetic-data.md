---
sidebar_position: 2
title: 'Isaac Sim Synthetic Data'
---

# Isaac Sim Synthetic Data

NVIDIA Isaac Sim is a robotics simulation application that provides photorealistic simulation capabilities and synthetic data generation for training AI models.

## Isaac Sim Architecture

Isaac Sim is built on NVIDIA Omniverse, providing:
- Physically accurate simulation
- Photorealistic rendering
- GPU-accelerated physics
- Real-time collaboration

## Synthetic Data Generation

### Sensor Simulation
Isaac Sim provides realistic sensor simulation including:
- RGB cameras
- Depth sensors
- LiDAR
- IMU
- Force/torque sensors

### Domain Randomization
To improve model generalization, Isaac Sim supports domain randomization:

```python
# Example of domain randomization in Isaac Sim
from omni.isaac.core.utils.prims import randomize_light_settings
from omni.isaac.core.utils.scene import randomize_material_properties

# Randomize lighting conditions
randomize_light_settings(light_prim_path="/World/Light",
                       intensity_range=(100, 1000),
                       color_temperature_range=(4000, 7000))

# Randomize material properties
randomize_material_properties(material_path="/World/Materials",
                            roughness_range=(0.1, 0.9),
                            metallic_range=(0.0, 1.0))
```

## USD (Universal Scene Description)

Isaac Sim uses USD as its scene description format:
- Hierarchical scene representation
- Multi-asset composition
- Layering and referencing
- Animation and simulation data

### USD Prim Structure
```usd
def Xform "Robot" (
    prepend apiSchemas = ["MotionVectorAPI"]
)
{
    def Xform "Base"
    {
        def Cylinder "Cylinder"
        {
            uniform token physics:shapeType = "cylinder"
        }
    }
}
```

## Replicator

NVIDIA Replicator is Isaac Sim's synthetic data generation framework:

```python
import omni.replicator.core as rep

# Create a camera and randomize its position
with rep.new_layer():
    camera = rep.create.camera()
    camera.set_position(rep.distribution.uniform((-100, -100, -100), (100, 100, 100)))

    # Randomize lighting
    lights = rep.create.light(light_type="distant", position=rep.distribution.uniform((-100, -100, -100), (100, 100, 100)))

    # Generate dataset
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(output_dir="output", rgb=True, depth=True)
    writer.write()
```

## Best Practices

- Use domain randomization to improve model generalization
- Validate synthetic data against real data
- Optimize scene complexity for training speed
- Implement proper data labeling and annotation
- Use realistic sensor noise models