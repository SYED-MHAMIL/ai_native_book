import React from 'react';
import { LandingPage } from '../components';

function Homepage() {
  // Sample modules data for the landing page
  const modules = [
    {
      title: "Module 1: The Robotic Nervous System (ROS 2)",
      description: "Middleware for robot control. Learn ROS 2 Nodes, Topics, Services, Python agents bridged to ROS controllers via rclpy, and URDF for humanoid robots.",
      lessons: 15,
      duration: "4 weeks",
      level: "Beginner",
      progress: 30,
      tags: ["ROS 2", "Nodes", "Topics", "Services"],
      onClick: () => window.location.href = '/docs/module-1-ros2'
    },
    {
      title: "Module 2: The Digital Twin (Gazebo & Unity)",
      description: "Physics simulation and environments. Explore gravity and collision simulation in Gazebo, high-fidelity Unity interaction, and sensor simulation (LiDAR, Depth, IMU).",
      lessons: 12,
      duration: "3 weeks",
      level: "Intermediate",
      progress: 0,
      tags: ["Gazebo", "Unity", "Simulation", "Physics"],
      onClick: () => window.location.href = '/docs/module-2-digital-twin'
    },
    {
      title: "Module 3: The AI-Robot Brain (NVIDIA Isaac)",
      description: "Perception and motion intelligence. Master Isaac Sim synthetic data and photoreal simulation, Isaac ROS accelerated VSLAM and navigation, and Nav2 for humanoid path planning.",
      lessons: 18,
      duration: "5 weeks",
      level: "Advanced",
      progress: 65,
      tags: ["Isaac", "AI", "Navigation", "VSLAM"],
      onClick: () => window.location.href = '/docs/module-3-isaac'
    },
    {
      title: "Module 4: Vision-Language-Action (VLA)",
      description: "Multimodal reasoning and behavior. Understand VLA concept overview, its role in humanoid interaction, and real-world application scenarios.",
      lessons: 10,
      duration: "2 weeks",
      level: "Advanced",
      progress: 0,
      tags: ["VLA", "Vision", "Language", "Action"],
      onClick: () => window.location.href = '/docs/module-4-vla'
    }
  ];

  return (
    <LandingPage
      title="robot.ai - Educational Robotics Platform"
      description="A comprehensive learning platform for humanoid robotics, combining ROS 2, simulation, AI, and multimodal systems."
      modules={modules}
      ctaText="Start Learning"
      heroTitle="Master Robotics with Structured Learning"
      heroSubtitle="Comprehensive educational modules covering ROS 2, Gazebo, NVIDIA Isaac, and Vision Language Actions"
    />
  );
}

export default Homepage;