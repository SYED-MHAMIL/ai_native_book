---
sidebar_position: 1
title: "Installation & Setup Guide"
description: "Complete setup guide for the Physical AI & Humanoid Robotics development environment"
tags: [setup, installation, environment, prerequisites]
---

# Installation & Setup Guide

This guide will help you set up the complete development environment for the Physical AI & Humanoid Robotics book. Follow these steps to get your system ready for the hands-on exercises and projects.

## System Requirements

Before starting the installation, ensure your system meets the following requirements:

### Minimum Requirements
- **OS**: Ubuntu 20.04/22.04 LTS, Windows 10/11 (WSL2), or macOS 10.15+
- **RAM**: 8 GB (16 GB recommended)
- **Storage**: 20 GB free space
- **Processor**: Multi-core processor with support for virtualization

### Recommended Hardware
For optimal performance, especially with simulation:
- **GPU**: NVIDIA RTX 3060 or better (for NVIDIA Isaac)
- **RAM**: 32 GB or more
- **SSD**: Fast storage for simulation assets

## Prerequisites Installation

### 1. Install Git
```bash
# Ubuntu/Debian
sudo apt update
sudo apt install git

# Windows (using Git Bash or WSL2)
# Download from https://git-scm.com/download/win

# macOS
xcode-select --install
```

### 2. Install Node.js (for development tools)
```bash
# Using Node Version Manager (recommended)
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.0/install.sh | bash
source ~/.bashrc
nvm install --lts
nvm use --lts
```

### 3. Install Python and pip
```bash
# Ubuntu/Debian
sudo apt install python3 python3-pip python3-venv

# Windows (using WSL2)
sudo apt install python3 python3-pip python3-venv

# macOS
brew install python3
```

## Module-Specific Setup

### ROS 2 Installation (Humble Hawksbill)

For the ROS 2 module, install the full desktop version:

```bash
# Ubuntu 22.04
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep2
sudo apt install python3-colcon-common-extensions
```

Set up the ROS 2 environment:
```bash
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Gazebo Simulation Setup

Install Gazebo Garden:
```bash
curl -sSL http://get.gazebosim.org | sh
```

### NVIDIA Isaac Setup (Optional)

For advanced AI robotics development:
1. Install NVIDIA drivers (535 or later)
2. Install CUDA Toolkit 12.x
3. Install cuDNN 8.x
4. Install Isaac ROS packages (detailed in Module 3)

## Development Environment Setup

### 1. Clone the Repository
```bash
git clone https://github.com/mohamil/ai-book-hack.git
cd ai-book-hack
```

### 2. Set up Python Virtual Environment
```bash
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install --upgrade pip
```

### 3. Install Additional Tools
```bash
# For documentation generation
pip install mkdocs
pip install python-frontmatter

# For AI integration
pip install openai
pip install anthropic
pip install transformers
```

## Testing Your Setup

### Test ROS 2 Installation
```bash
source /opt/ros/humble/setup.bash
ros2 topic list
```

### Test Python Environment
```bash
python3 -c "import rclpy; print('ROS 2 Python bindings work!')"
```

### Test Documentation Server
```bash
cd website
npm start
```

## Troubleshooting

### Common Issues

**Issue**: `command 'ros2' not found`
**Solution**: Ensure you've sourced the ROS 2 setup file or added it to your `.bashrc`

**Issue**: Permission errors with Docker (for simulation containers)
**Solution**: Add your user to the docker group: `sudo usermod -aG docker $USER`

**Issue**: GPU not detected for Isaac
**Solution**: Verify NVIDIA drivers: `nvidia-smi`

## Next Steps

Once you've completed the installation:

1. Verify all components work with the [Environment Verification](./environment-verification.md) guide
2. Explore the [ROS 2 Module](../modules/01-ros2/intro.md) to start with the fundamentals
3. Join our [Community Discord](https://discord.gg/docusaurus) for support and discussions

---

**Note**: Some modules have additional requirements. Detailed setup instructions for each module are provided in their respective sections.