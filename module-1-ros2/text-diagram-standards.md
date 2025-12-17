# Text-Based Diagram Standards

## ASCII Art Guidelines

### General Principles
- Use consistent character sets: ┌ ┐ └ ┘ │ ─ ├ ┤ ┬ ┴ ┼ for boxes
- Use arrows: → ← ↑ ↓ for directional flow
- Use consistent spacing for alignment
- Keep diagrams simple and clear
- Add explanatory text below complex diagrams

### Common Diagram Patterns

#### System Architecture
```
┌─────────────────┐    ┌──────────────────────┐    ┌──────────────────┐
│   Component A   │───▶│     Component B      │───▶│   Component C    │
└─────────────────┘    └──────────────────────┘    └──────────────────┘
        │                       │                          │
        ▼                       ▼                          ▼
┌─────────────────┐    ┌──────────────────────┐    ┌──────────────────┐
│   Data Flow 1   │    │     Data Flow 2      │    │   Data Flow 3    │
└─────────────────┘    └──────────────────────┘    └──────────────────┘
```

#### Process Flow
```
Start → Condition → Action → [Loop Back/End]
         │           │
         ▼           ▼
      Alternative  Result
```

#### Data Structure
```
┌─────────────────────────────────┐
│           Data Structure        │
├─────────────────────────────────┤
│ Field 1: [description]          │
│ Field 2: [description]          │
│ Field 3: [description]          │
└─────────────────────────────────┘
```

### Humanoid-Specific Diagrams
- Use consistent terminology for robot parts
- Show data flow between sensors, controllers, and actuators
- Illustrate coordinate systems and frame relationships
- Represent ROS 2 communication patterns clearly

### Best Practices
- Keep diagrams under 10 lines when possible
- Use comments to explain complex diagrams
- Align elements properly for readability
- Use consistent symbols for similar concepts