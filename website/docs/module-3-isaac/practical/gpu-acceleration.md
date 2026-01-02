---
sidebar_position: 1
title: "Practical GPU Acceleration for Robotics"
description: "Hands-on guide to implementing GPU-accelerated perception and AI systems using NVIDIA Isaac"
tags: [nvidia-isaac, gpu-acceleration, practical, perception]
---

# Practical GPU Acceleration for Robotics

## Overview

This practical guide will walk you through implementing GPU-accelerated perception and AI systems for humanoid robots using NVIDIA Isaac. You'll create actual accelerated nodes, optimize models for inference, and test them with real robotic data.

## Prerequisites

Before starting this practical session, ensure you have:

- NVIDIA GPU with CUDA support (RTX series recommended)
- CUDA Toolkit 11.8 or later installed
- cuDNN library installed
- NVIDIA Isaac ROS packages installed
- Basic understanding of GPU computing concepts
- ROS 2 Humble with Isaac ROS extensions

## Setting Up Your GPU Environment

### Installing NVIDIA Isaac ROS Packages

```bash
# Update package lists
sudo apt update

# Install Isaac ROS packages
sudo apt install ros-humble-isaac-ros-dev
sudo apt install ros-humble-isaac-ros-visual-slam
sudo apt install ros-humble-isaac-ros-object-detection
sudo apt install ros-humble-isaac-ros-segmentation
sudo apt install ros-humble-isaac-ros-bit-matrix

# Install additional GPU acceleration packages
sudo apt install ros-humble-isaac-ros-gxf
sudo apt install ros-humble-isaac-ros-ess
sudo apt install ros-humble-isaac-ros-occupancy-grid-localizer
```

### Verifying GPU Setup

```bash
# Check NVIDIA GPU
nvidia-smi

# Check CUDA installation
nvcc --version

# Test CUDA with Python
python3 -c "import torch; print(f'CUDA available: {torch.cuda.is_available()}'); print(f'CUDA devices: {torch.cuda.device_count()}'); print(f'Current device: {torch.cuda.current_device()}')"
```

## Creating Your First GPU-Accelerated Node

### Setting Up the Package

```bash
cd ~/ros2_humanoid_ws/src
ros2 pkg create --build-type ament_python gpu_perception --dependencies rclpy sensor_msgs cv_bridge message_filters
```

### Creating a GPU-Accelerated Image Processing Node

Create `gpu_perception/gpu_perception/gpu_image_processor.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
import torch.nn as nn
from torchvision import transforms

class GPUImageProcessor(Node):
    def __init__(self):
        super().__init__('gpu_image_processor')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Check for GPU availability
        if torch.cuda.is_available():
            self.device = torch.device('cuda')
            self.get_logger().info(f'Using GPU: {torch.cuda.get_device_name(0)}')
        else:
            self.device = torch.device('cpu')
            self.get_logger().warning('GPU not available, using CPU')

        # Create subscribers and publishers
        self.image_sub = self.create_subscription(
            Image, 'input_image', self.image_callback, 10)
        self.processed_pub = self.create_publisher(
            Image, 'processed_image', 10)

        # Initialize a simple CNN model for demonstration
        self.model = self.create_simple_model().to(self.device)
        self.model.eval()

        # Preprocessing transforms
        self.transform = transforms.Compose([
            transforms.ToTensor(),
        ])

        self.get_logger().info('GPU Image Processor Node Started')

    def create_simple_model(self):
        """Create a simple CNN model for image processing"""
        class SimpleCNN(nn.Module):
            def __init__(self):
                super(SimpleCNN, self).__init__()
                self.conv1 = nn.Conv2d(3, 16, 3, padding=1)
                self.pool = nn.MaxPool2d(2, 2)
                self.conv2 = nn.Conv2d(16, 32, 3, padding=1)
                self.conv3 = nn.Conv2d(32, 3, 3, padding=1)  # Output 3 channels

            def forward(self, x):
                x = self.pool(torch.relu(self.conv1(x)))
                x = self.pool(torch.relu(self.conv2(x)))
                x = torch.sigmoid(self.conv3(x))
                return x

        return SimpleCNN()

    def image_callback(self, msg):
        """Process image using GPU acceleration"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Convert to tensor and move to GPU
            tensor_image = self.transform(cv_image).unsqueeze(0).to(self.device)

            # Normalize to [0, 1] range
            tensor_image = tensor_image.float() / 255.0

            # Process with GPU-accelerated model
            with torch.no_grad():
                processed_tensor = self.model(tensor_image)

            # Convert back to [0, 255] and detach from GPU
            processed_tensor = processed_tensor.squeeze(0) * 255.0
            processed_image = processed_tensor.permute(1, 2, 0).cpu().numpy().astype(np.uint8)

            # Convert back to ROS image message
            result_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding='bgr8')
            result_msg.header = msg.header  # Preserve header

            # Publish the processed image
            self.processed_pub.publish(result_msg)

            self.get_logger().info('Image processed with GPU acceleration')

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    processor = GPUImageProcessor()

    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    finally:
        processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating a GPU-Accelerated Object Detection Node

Create `gpu_perception/gpu_perception/gpu_object_detector.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import torch
import torchvision
from torchvision import transforms
import numpy as np

class GPUObjectDetector(Node):
    def __init__(self):
        super().__init__('gpu_object_detector')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Check for GPU availability
        if torch.cuda.is_available():
            self.device = torch.device('cuda')
            self.get_logger().info(f'Using GPU: {torch.cuda.get_device_name(0)}')
        else:
            self.device = torch.device('cpu')
            self.get_logger().warning('GPU not available, using CPU')

        # Create subscribers and publishers
        self.image_sub = self.create_subscription(
            Image, 'input_image', self.image_callback, 10)
        self.detection_pub = self.create_publisher(
            Detection2DArray, 'object_detections', 10)

        # Load pre-trained model (YOLOv5 or similar)
        self.model = self.load_detection_model()
        self.model.eval()

        # COCO dataset class names for YOLO
        self.coco_names = [
            'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train',
            'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign',
            'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep',
            'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella',
            'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard',
            'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard',
            'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork',
            'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
            'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair',
            'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv',
            'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave',
            'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase',
            'scissors', 'teddy bear', 'hair drier', 'toothbrush'
        ]

        self.get_logger().info('GPU Object Detector Node Started')

    def load_detection_model(self):
        """Load a pre-trained object detection model"""
        # Using torchvision's pre-trained model as an example
        # In practice, you might use YOLOv5, YOLOv8, or custom models
        model = torchvision.models.detection.fasterrcnn_resnet50_fpn(weights='DEFAULT')
        model = model.to(self.device)
        return model

    def image_callback(self, msg):
        """Process image for object detection using GPU"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Preprocess image
            transform = transforms.Compose([
                transforms.ToPILImage(),
                transforms.ToTensor(),
            ])

            # Convert BGR to RGB for PyTorch
            cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            input_tensor = transform(cv_image_rgb).unsqueeze(0).to(self.device)

            # Run inference
            with torch.no_grad():
                predictions = self.model(input_tensor)

            # Process predictions
            detections = self.process_predictions(predictions[0], cv_image.shape, msg.header)

            # Publish detections
            self.detection_pub.publish(detections)

            self.get_logger().info(f'Detected {len(detections.detections)} objects')

        except Exception as e:
            self.get_logger().error(f'Error in object detection: {str(e)}')

    def process_predictions(self, prediction, img_shape, header):
        """Process model predictions into Detection2DArray message"""
        detections = Detection2DArray()
        detections.header = header

        # Get prediction data
        boxes = prediction['boxes'].cpu().numpy()
        scores = prediction['scores'].cpu().numpy()
        labels = prediction['labels'].cpu().numpy()

        height, width = img_shape[:2]

        for i in range(len(boxes)):
            if scores[i] > 0.5:  # Confidence threshold
                box = boxes[i]
                x_min, y_min, x_max, y_max = box

                # Create detection message
                detection = Detection2D()
                detection.header = header

                # Set bounding box
                detection.bbox.size_x = float(x_max - x_min)
                detection.bbox.size_y = float(y_max - y_min)
                detection.bbox.center.x = float((x_min + x_max) / 2)
                detection.bbox.center.y = float((y_min + y_max) / 2)

                # Set results
                result = ObjectHypothesisWithPose()
                result.hypothesis.class_id = str(self.coco_names[labels[i]-1]) if labels[i]-1 < len(self.coco_names) else f'unknown_{labels[i]}'
                result.hypothesis.score = float(scores[i])

                detection.results.append(result)
                detections.detections.append(detection)

        return detections

def main(args=None):
    rclpy.init(args=args)
    detector = GPUObjectDetector()

    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Updating Package Entry Points

Edit `gpu_perception/setup.py`:

```python
import os
from glob import glob
from setuptools import setup

package_name = 'gpu_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='GPU-accelerated perception nodes for robotics',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gpu_image_processor = gpu_perception.gpu_image_processor:main',
            'gpu_object_detector = gpu_perception.gpu_object_detector:main',
        ],
    },
)
```

## Creating TensorRT Optimization Node

Create `gpu_perception/gpu_perception/tensorrt_optimizer.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
import torch_tensorrt
import torchvision.models as models
import numpy as np

class TensorRTOptimizer(Node):
    def __init__(self):
        super().__init__('tensorrt_optimizer')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Check for GPU availability
        if torch.cuda.is_available():
            self.device = torch.device('cuda')
            self.get_logger().info(f'Using GPU: {torch.cuda.get_device_name(0)}')
        else:
            self.get_logger().error('TensorRT requires GPU')
            return

        # Create subscribers and publishers
        self.image_sub = self.create_subscription(
            Image, 'input_image', self.image_callback, 10)
        self.optimized_pub = self.create_publisher(
            Image, 'optimized_output', 10)

        # Load and optimize model with TensorRT
        self.model = self.load_and_optimize_model()
        self.model.eval()

        self.get_logger().info('TensorRT Optimizer Node Started')

    def load_and_optimize_model(self):
        """Load model and optimize with TensorRT"""
        # Load a pre-trained model
        model = models.resnet18(weights='DEFAULT')
        model = model.to(self.device)
        model.eval()

        # Define input shape for optimization
        input_shape = (1, 3, 224, 224)  # Batch size 1, 3 channels, 224x224

        # Convert to TorchScript
        example_input = torch.randn(input_shape).to(self.device)
        traced_model = torch.jit.trace(model, example_input)

        # Optimize with TensorRT
        try:
            optimized_model = torch_tensorrt.compile(
                traced_model,
                inputs=[torch_tensorrt.Input(input_shape)],
                enabled_precisions={torch.float, torch.half},  # Use FP32 and FP16
                workspace_size=1 << 20,  # 1MB workspace
                debug=True
            )
            self.get_logger().info('Model optimized with TensorRT')
            return optimized_model
        except Exception as e:
            self.get_logger().warning(f'TensorRT optimization failed: {str(e)}, using original model')
            return traced_model

    def image_callback(self, msg):
        """Process image using TensorRT-optimized model"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Resize image to model input size (224x224 for ResNet)
            cv_image_resized = cv2.resize(cv_image, (224, 224))

            # Convert to tensor and preprocess
            input_tensor = torch.from_numpy(cv_image_resized).float().permute(2, 0, 1).unsqueeze(0)
            input_tensor = input_tensor.to(self.device) / 255.0  # Normalize to [0,1]

            # Run inference with optimized model
            with torch.no_grad():
                output = self.model(input_tensor)

            # Process output (for this example, just return the input image)
            # In practice, you would use the model output for your specific task
            result_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            result_msg.header = msg.header

            # Publish the result
            self.optimized_pub.publish(result_msg)

            self.get_logger().info('Image processed with TensorRT-optimized model')

        except Exception as e:
            self.get_logger().error(f'Error processing with TensorRT: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    optimizer = TensorRTOptimizer()

    try:
        rclpy.spin(optimizer)
    except KeyboardInterrupt:
        pass
    finally:
        optimizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Add this to the entry points:

```python
entry_points={
    'console_scripts': [
        'gpu_image_processor = gpu_perception.gpu_image_processor:main',
        'gpu_object_detector = gpu_perception.gpu_object_detector:main',
        'tensorrt_optimizer = gpu_perception.tensorrt_optimizer:main',
    ],
},
```

## Creating a GPU Memory Management Node

Create `gpu_perception/gpu_perception/gpu_memory_manager.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
import gc

class GPUMemoryManager(Node):
    def __init__(self):
        super().__init__('gpu_memory_manager')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Check for GPU availability
        if torch.cuda.is_available():
            self.device = torch.device('cuda')
            self.get_logger().info(f'Using GPU: {torch.cuda.get_device_name(0)}')
        else:
            self.device = torch.device('cpu')
            self.get_logger().warning('GPU not available, using CPU')

        # Create subscribers and publishers
        self.image_sub = self.create_subscription(
            Image, 'input_image', self.image_callback, 10)
        self.processed_pub = self.create_publisher(
            Image, 'memory_managed_output', 10)

        # Initialize a model
        self.model = self.create_model()
        self.model = self.model.to(self.device)
        self.model.eval()

        # Memory management parameters
        self.process_count = 0
        self.memory_check_interval = 10  # Check memory every 10 processes

        self.get_logger().info('GPU Memory Manager Node Started')

    def create_model(self):
        """Create a simple model for processing"""
        import torch.nn as nn

        class SimpleModel(nn.Module):
            def __init__(self):
                super().__init__()
                self.conv = nn.Conv2d(3, 3, 3, padding=1)

            def forward(self, x):
                return torch.relu(self.conv(x))

        return SimpleModel()

    def image_callback(self, msg):
        """Process image with GPU memory management"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Convert to tensor and move to GPU
            input_tensor = torch.from_numpy(cv_image).float().permute(2, 0, 1).unsqueeze(0)
            input_tensor = input_tensor.to(self.device) / 255.0

            # Process with model
            with torch.no_grad():
                output_tensor = self.model(input_tensor)

            # Convert back to image
            output_tensor = output_tensor.squeeze(0) * 255.0
            output_image = output_tensor.permute(1, 2, 0).cpu().numpy().astype(np.uint8)

            # Convert back to ROS image message
            result_msg = self.bridge.cv2_to_imgmsg(output_image, encoding='bgr8')
            result_msg.header = msg.header

            # Publish result
            self.processed_pub.publish(result_msg)

            self.process_count += 1

            # Periodic memory cleanup
            if self.process_count % self.memory_check_interval == 0:
                self.cleanup_gpu_memory()

            self.get_logger().info(f'Processed image {self.process_count} with memory management')

        except Exception as e:
            self.get_logger().error(f'Error in GPU memory management: {str(e)}')

    def cleanup_gpu_memory(self):
        """Clean up GPU memory"""
        # Clear PyTorch cache
        if torch.cuda.is_available():
            torch.cuda.empty_cache()

        # Force Python garbage collection
        gc.collect()

        # Log memory usage
        if torch.cuda.is_available():
            memory_allocated = torch.cuda.memory_allocated()
            memory_reserved = torch.cuda.memory_reserved()
            self.get_logger().info(
                f'GPU Memory - Allocated: {memory_allocated / 1024**2:.1f} MB, '
                f'Reserved: {memory_reserved / 1024**2:.1f} MB'
            )

def main(args=None):
    rclpy.init(args=args)
    manager = GPUMemoryManager()

    try:
        rclpy.spin(manager)
    except KeyboardInterrupt:
        pass
    finally:
        manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Add this to the entry points as well:

```python
entry_points={
    'console_scripts': [
        'gpu_image_processor = gpu_perception.gpu_image_processor:main',
        'gpu_object_detector = gpu_perception.gpu_object_detector:main',
        'tensorrt_optimizer = gpu_perception.tensorrt_optimizer:main',
        'gpu_memory_manager = gpu_perception.gpu_memory_manager:main',
    ],
},
```

## Building and Testing Your Nodes

### Installing Dependencies

First, install the required Python packages:

```bash
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip3 install opencv-python

# If using TensorRT optimization
pip3 install torch-tensorrt
```

### Building the Package

```bash
cd ~/ros2_humanoid_ws
colcon build --packages-select gpu_perception
source install/setup.bash
```

## Testing GPU Acceleration

### Creating a Test Launch File

Create `gpu_perception/launch/gpu_perception_test.launch.py`:

```python
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch GPU image processor
    gpu_processor = ExecuteProcess(
        cmd=['ros2', 'run', 'gpu_perception', 'gpu_image_processor'],
        output='screen'
    )

    # Launch GPU object detector
    gpu_detector = ExecuteProcess(
        cmd=['ros2', 'run', 'gpu_perception', 'gpu_object_detector'],
        output='screen'
    )

    # Launch TensorRT optimizer
    tensorrt_optimizer = ExecuteProcess(
        cmd=['ros2', 'run', 'gpu_perception', 'tensorrt_optimizer'],
        output='screen'
    )

    return LaunchDescription([
        gpu_processor,
        gpu_detector,
        tensorrt_optimizer
    ])
```

### Running Performance Tests

Create a performance testing script `gpu_perception/test/performance_test.py`:

```python
#!/usr/bin/env python3

import time
import torch
import torch.nn as nn
import numpy as np
from torchvision import transforms

def benchmark_gpu_operations():
    """Benchmark GPU operations for robotics perception"""

    if not torch.cuda.is_available():
        print("GPU not available, skipping benchmarks")
        return

    device = torch.device('cuda')
    print(f"Benchmarking on GPU: {torch.cuda.get_device_name(0)}")

    # Test 1: Tensor operations
    print("\n1. Tensor Operations Benchmark")
    tensor_size = (1, 3, 480, 640)  # Typical image size

    # CPU operations
    start_time = time.time()
    cpu_tensor = torch.randn(*tensor_size)
    cpu_ops_time = time.time() - start_time

    # GPU operations
    start_time = time.time()
    gpu_tensor = torch.randn(*tensor_size).to(device)
    gpu_ops_time = time.time() - start_time

    print(f"  CPU tensor creation: {cpu_ops_time*1000:.2f} ms")
    print(f"  GPU tensor creation: {gpu_ops_time*1000:.2f} ms")

    # Test 2: Neural network inference
    print("\n2. Neural Network Inference Benchmark")

    class SimpleCNN(nn.Module):
        def __init__(self):
            super().__init__()
            self.conv1 = nn.Conv2d(3, 16, 3, padding=1)
            self.conv2 = nn.Conv2d(16, 32, 3, padding=1)
            self.conv3 = nn.Conv2d(32, 3, 3, padding=1)

        def forward(self, x):
            x = torch.relu(self.conv1(x))
            x = torch.relu(self.conv2(x))
            x = torch.sigmoid(self.conv3(x))
            return x

    # CPU model
    cpu_model = SimpleCNN()
    cpu_tensor = torch.randn(*tensor_size)

    start_time = time.time()
    with torch.no_grad():
        cpu_result = cpu_model(cpu_tensor)
    cpu_inference_time = time.time() - start_time

    # GPU model
    gpu_model = SimpleCNN().to(device)
    gpu_tensor = torch.randn(*tensor_size).to(device)

    # Warm up GPU
    with torch.no_grad():
        _ = gpu_model(gpu_tensor)

    start_time = time.time()
    with torch.no_grad():
        gpu_result = gpu_model(gpu_tensor)
    gpu_inference_time = time.time() - start_time

    print(f"  CPU inference: {cpu_inference_time*1000:.2f} ms")
    print(f"  GPU inference: {gpu_inference_time*1000:.2f} ms")
    print(f"  Speedup: {cpu_inference_time/gpu_inference_time:.2f}x")

    # Test 3: Memory bandwidth
    print("\n3. Memory Bandwidth Test")

    # Test large tensor transfer
    large_tensor = torch.randn(100, 3, 480, 640)  # Large batch

    start_time = time.time()
    large_gpu_tensor = large_tensor.to(device)
    transfer_time = time.time() - start_time

    print(f"  Large tensor transfer (100 images): {transfer_time*1000:.2f} ms")
    print(f"  Transfer rate: {(large_tensor.numel() * 4) / (transfer_time * 1024**3):.2f} GB/s")

def test_gpu_memory():
    """Test GPU memory management"""
    if not torch.cuda.is_available():
        print("GPU not available")
        return

    print(f"\n4. GPU Memory Information")
    print(f"  Total memory: {torch.cuda.get_device_properties(0).total_memory / 1024**3:.2f} GB")
    print(f"  Memory allocated: {torch.cuda.memory_allocated() / 1024**2:.2f} MB")
    print(f"  Memory reserved: {torch.cuda.memory_reserved() / 1024**2:.2f} MB")

if __name__ == '__main__':
    print("=== GPU Acceleration Performance Test ===")
    benchmark_gpu_operations()
    test_gpu_memory()
    print("\n=== Benchmark Complete ===")
```

## Running the Performance Test

```bash
cd ~/ros2_humanoid_ws
source install/setup.bash
python3 src/gpu_perception/test/performance_test.py
```

## Isaac ROS Integration Example

Create a more advanced Isaac ROS integration example `gpu_perception/gpu_perception/isaac_ros_integration.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import torch
import numpy as np

class IsaacROSIntegration(Node):
    def __init__(self):
        super().__init__('isaac_ros_integration')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Check for GPU availability
        if torch.cuda.is_available():
            self.device = torch.device('cuda')
            self.get_logger().info(f'Using GPU: {torch.cuda.get_device_name(0)}')
        else:
            self.device = torch.device('cpu')
            self.get_logger().warning('GPU not available, using CPU')

        # Subscribe to Isaac ROS camera topics
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)

        # Publishers for processed data
        self.depth_pub = self.create_publisher(Image, 'gpu_depth_map', 10)
        self.semantic_pub = self.create_publisher(Image, 'gpu_semantic_map', 10)

        # Initialize Isaac-style processing pipeline
        self.initialize_pipeline()

        self.get_logger().info('Isaac ROS GPU Integration Node Started')

    def initialize_pipeline(self):
        """Initialize the GPU-accelerated processing pipeline"""
        # This would integrate with Isaac ROS components in a real system
        # For this example, we'll create placeholder models
        self.depth_model = self.create_depth_model().to(self.device)
        self.segmentation_model = self.create_segmentation_model().to(self.device)

        self.depth_model.eval()
        self.segmentation_model.eval()

    def create_depth_model(self):
        """Create a simple depth estimation model"""
        import torch.nn as nn

        class SimpleDepthNet(nn.Module):
            def __init__(self):
                super().__init__()
                self.conv1 = nn.Conv2d(3, 64, 3, padding=1)
                self.conv2 = nn.Conv2d(64, 128, 3, padding=1)
                self.conv3 = nn.Conv2d(128, 1, 3, padding=1)

            def forward(self, x):
                x = torch.relu(self.conv1(x))
                x = torch.relu(self.conv2(x))
                x = torch.sigmoid(self.conv3(x))  # Depth in [0,1]
                return x

        return SimpleDepthNet()

    def create_segmentation_model(self):
        """Create a simple segmentation model"""
        import torch.nn as nn

        class SimpleSegNet(nn.Module):
            def __init__(self):
                super().__init__()
                self.conv1 = nn.Conv2d(3, 64, 3, padding=1)
                self.conv2 = nn.Conv2d(64, 32, 3, padding=1)
                self.conv3 = nn.Conv2d(32, 8, 3, padding=1)  # 8 classes

            def forward(self, x):
                x = torch.relu(self.conv1(x))
                x = torch.relu(self.conv2(x))
                x = torch.softmax(self.conv3(x), dim=1)  # Class probabilities
                return x

        return SimpleSegNet()

    def image_callback(self, msg):
        """Process image with Isaac-style GPU acceleration"""
        try:
            # Convert ROS image to tensor
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            input_tensor = torch.from_numpy(cv_image).float().permute(2, 0, 1).unsqueeze(0)
            input_tensor = input_tensor.to(self.device) / 255.0

            # Run depth estimation
            with torch.no_grad():
                depth_map = self.depth_model(input_tensor)

            # Run semantic segmentation
            with torch.no_grad():
                segmentation = self.segmentation_model(input_tensor)
                segmentation = torch.argmax(segmentation, dim=1)  # Get class indices

            # Convert results back to ROS messages
            depth_image = (depth_map.squeeze(0).squeeze(0).cpu().numpy() * 255).astype(np.uint8)
            seg_image = (segmentation.squeeze(0).cpu().numpy() * 32).astype(np.uint8)  # Scale for visibility

            # Publish results
            depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding='mono8')
            depth_msg.header = msg.header
            self.depth_pub.publish(depth_msg)

            seg_msg = self.bridge.cv2_to_imgmsg(seg_image, encoding='mono8')
            seg_msg.header = msg.header
            self.semantic_pub.publish(seg_msg)

            self.get_logger().info('Isaac-style GPU processing completed')

        except Exception as e:
            self.get_logger().error(f'Error in Isaac ROS integration: {str(e)}')

    def camera_info_callback(self, msg):
        """Handle camera calibration info"""
        # In a real system, this would be used for geometric processing
        self.get_logger().info(f'Camera info received: {msg.k}')

def main(args=None):
    rclpy.init(args=args)
    integration_node = IsaacROSIntegration()

    try:
        rclpy.spin(integration_node)
    except KeyboardInterrupt:
        pass
    finally:
        integration_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Add this to the entry points:

```python
entry_points={
    'console_scripts': [
        'gpu_image_processor = gpu_perception.gpu_image_processor:main',
        'gpu_object_detector = gpu_perception.gpu_object_detector:main',
        'tensorrt_optimizer = gpu_perception.tensorrt_optimizer:main',
        'gpu_memory_manager = gpu_perception.gpu_memory_manager:main',
        'isaac_ros_integration = gpu_perception.isaac_ros_integration:main',
    ],
},
```

## Module Summary

In this practical session, you:

- Created GPU-accelerated perception nodes for robotics
- Implemented object detection with GPU acceleration
- Optimized models using TensorRT
- Managed GPU memory efficiently
- Integrated with Isaac ROS concepts
- Benchmarked GPU performance for robotics tasks

These practical implementations provide the foundation for building high-performance perception systems for humanoid robots.

## Next Steps

Continue to the [Advanced Topics](../advanced/index.md) section to learn about more sophisticated GPU acceleration techniques, including multi-GPU setups, real-time optimization, and deployment strategies for robotic platforms.