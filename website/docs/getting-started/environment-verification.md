---
sidebar_position: 2
title: "Environment Verification"
description: "Verify that your Physical AI & Humanoid Robotics development environment is properly configured"
tags: [setup, verification, testing, environment]
---

# Environment Verification

## Overview

This guide will help you verify that all components of your Physical AI & Humanoid Robotics development environment are properly installed and configured. Complete these verification steps before proceeding with the hands-on exercises.

## System Verification Steps

### 1. Verify ROS 2 Installation

Check that ROS 2 is properly installed and sourced:

```bash
# Source ROS 2 environment (if not in .bashrc)
source /opt/ros/humble/setup.bash

# Check ROS 2 version
ros2 --version

# Verify core ROS 2 commands work
ros2 topic list
ros2 node list
ros2 service list
```

Expected output: ROS 2 Humble Hawksbill version number and empty lists for topics/nodes/services (no errors).

### 2. Test Python ROS 2 Bindings

Verify that Python can access ROS 2 libraries:

```bash
python3 -c "import rclpy; print('ROS 2 Python bindings: OK')"
python3 -c "import rclpy.node; print('ROS 2 Node import: OK')"
python3 -c "import builtin_interfaces.msg; print('ROS 2 messages: OK')"
```

Expected output: No errors, confirmation messages for each import.

### 3. Test Basic ROS 2 Functionality

Run a simple publisher-subscriber test:

```bash
# Terminal 1: Start a publisher
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker

# Terminal 2: Start a subscriber
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

Expected output: Messages flowing from talker to listener without errors.

### 4. Verify Gazebo Installation

Test Gazebo simulation environment:

```bash
# Check Gazebo version
gz --version

# Launch a simple world (this may take a moment to start)
gz sim -r shapes.sdf
```

Expected output: Gazebo simulation window opens with a simple scene.

### 5. Check NVIDIA GPU Support (if applicable)

If you have an NVIDIA GPU for Isaac integration:

```bash
# Check NVIDIA driver
nvidia-smi

# Check CUDA installation
nvcc --version

# Test CUDA capability
python3 -c "import torch; print(f'CUDA available: {torch.cuda.is_available()}'); print(f'CUDA devices: {torch.cuda.device_count()}')"
```

Expected output: NVIDIA driver information, CUDA version, and PyTorch confirming CUDA availability.

### 6. Test Python Environment

Verify required Python packages are available:

```bash
# Check basic packages
python3 -c "import numpy; print(f'NumPy: {numpy.__version__}')"
python3 -c "import cv2; print(f'OpenCV: {cv2.__version__}')"
python3 -c "import yaml; print('PyYAML: OK')"

# Check AI packages
python3 -c "import transformers; print('Transformers: OK')"
python3 -c "import torch; print(f'PyTorch: {torch.__version__}')"
```

### 7. Test Git and Repository Access

Verify Git functionality:

```bash
# Check Git version
git --version

# Test Git configuration
git config --global user.name
git config --global user.email

# Test repository cloning (if you have access to the course repository)
git clone https://github.com/mohamil/ai-book-hack.git test-clone
ls test-clone
rm -rf test-clone
```

### 8. Test Development Tools

Verify development tools are accessible:

```bash
# Check Node.js and npm
node --version
npm --version

# Test Python virtual environment (if set up)
source venv/bin/activate  # or venv\Scripts\activate on Windows
pip list | grep -E "(rclpy|ros2)"
deactivate
```

## Module-Specific Verification

### ROS 2 Module Verification

Test ROS 2 specific functionality:

```bash
# Create a test workspace
mkdir -p ~/ros2_test_ws/src
cd ~/ros2_test_ws

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash

# Verify workspace setup
printenv | grep ROS
```

### Simulation Module Verification

Test simulation capabilities:

```bash
# Check Gazebo models are available
ls /usr/share/gazebo-*/models | head -10

# Test launching a robot model (if available)
# This assumes you have robot models installed
ros2 launch gazebo_ros empty_world.launch.py
```

## Network and Communication Verification

### Check ROS 2 Network Setup

```bash
# Test ROS 2 domain (should be isolated)
echo $ROS_DOMAIN_ID

# Test multicast communication
# This command should show no errors
ros2 topic pub /test_topic std_msgs/String "data: 'test'" --once
```

### Check for Common Issues

Run a diagnostic script to identify common problems:

```bash
#!/bin/bash
# environment_diagnostic.sh

echo "=== Physical AI & Humanoid Robotics Environment Diagnostic ==="

echo -e "\n1. ROS 2 Installation:"
if command -v ros2 &> /dev/null; then
    echo "✓ ROS 2 installed: $(ros2 --version)"
else
    echo "✗ ROS 2 not found"
fi

echo -e "\n2. ROS 2 Environment:"
if [ -n "$ROS_DISTRO" ]; then
    echo "✓ ROS 2 sourced: $ROS_DISTRO"
else
    echo "✗ ROS 2 not sourced - run: source /opt/ros/humble/setup.bash"
fi

echo -e "\n3. Python ROS 2 Bindings:"
python3 -c "import rclpy" 2>/dev/null
if [ $? -eq 0 ]; then
    echo "✓ Python ROS 2 bindings available"
else
    echo "✗ Python ROS 2 bindings not available"
fi

echo -e "\n4. Gazebo Installation:"
if command -v gz &> /dev/null; then
    echo "✓ Gazebo installed: $(gz --version)"
else
    echo "✗ Gazebo not found"
fi

echo -e "\n5. Python Packages:"
python3 -c "import cv2" 2>/dev/null
if [ $? -eq 0 ]; then
    echo "✓ OpenCV available"
else
    echo "✗ OpenCV not available"
fi

echo -e "\n=== Diagnostic Complete ==="
```

## Troubleshooting Common Issues

### ROS 2 Not Found
- Ensure you've sourced the ROS 2 setup script
- Check that ROS 2 Humble is properly installed
- Verify the installation path matches your sourcing command

### Python Import Errors
- Ensure you're using Python 3
- Check that your Python path includes ROS 2 packages
- Verify virtual environment is properly configured

### GPU Not Detected
- Verify NVIDIA drivers are properly installed
- Check CUDA installation and version compatibility
- Ensure Isaac ROS packages are installed

## Performance Verification

### Benchmark System Performance

Run basic performance tests to ensure your system can handle robotics workloads:

```bash
# CPU benchmark
python3 -c "
import time
import numpy as np
start = time.time()
a = np.random.random((1000, 1000))
b = np.random.random((1000, 1000))
c = np.dot(a, b)
elapsed = time.time() - start
print(f'NumPy matrix multiplication: {elapsed:.2f} seconds')
"

# Memory check
free -h
```

## Verification Checklist

Complete this checklist to ensure your environment is ready:

- [ ] ROS 2 Humble installed and working
- [ ] Python ROS 2 bindings accessible
- [ ] Basic ROS 2 functionality tested
- [ ] Gazebo simulation environment working
- [ ] Python packages installed and accessible
- [ ] Git and version control working
- [ ] Network communication verified
- [ ] System performance adequate for robotics work

## Next Steps

Once you've completed all verification steps:

1. **Proceed to Module 1**: Explore the [ROS 2 Module](../modules/01-ros2/intro.md) to begin with the fundamentals
2. **Join Community**: Connect with other learners in our [Community Discord](https://discord.gg/docusaurus)
3. **Report Issues**: If you encounter any problems, document them and seek help

Your development environment is now verified and ready for the Physical AI & Humanoid Robotics course!