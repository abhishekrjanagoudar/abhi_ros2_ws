#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
import torch
import torch.nn as nn
import numpy as np

# Phase 3: Inference Node
# This node loads the trained model and publishes velocity commands based on LiDAR input.

class RobotDriverNet(nn.Module):
    """ Same model architecture as used in Phase 2 Training """
    def __init__(self, input_size=360, hidden_sizes=[256, 128, 64], output_size=2):
        super(RobotDriverNet, self).__init__()
        self.network = nn.Sequential(
            nn.Linear(input_size, hidden_sizes[0]),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(hidden_sizes[0], hidden_sizes[1]),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(hidden_sizes[1], hidden_sizes[2]),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(hidden_sizes[2], output_size)
        )

    def forward(self, x):
        return self.network(x)

class AIDriverNode(Node):
    def __init__(self):
        super().__init__('ai_driver_node')
        
        # Load the PyTorch model
        self.model = RobotDriverNet()
        model_path = '/home/abhi_ros2_ws/robot_driver_model.pth'
        
        try:
            self.model.load_state_dict(torch.load(model_path, map_location=torch.device('cpu')))
            self.model.eval()
            self.get_logger().info(f"Successfully loaded model from {model_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            raise e
            
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/diff_cont/cmd_vel', 10)
        
        # Subscriber for LiDAR scans
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.get_logger().info("AI Driver Node is running. Publishing to /diff_cont/cmd_vel...")

    def scan_callback(self, msg):
        import math
        # 1. Clean the LiDAR data (exact match to Phase 1 Data Collection)
        max_r = msg.range_max if math.isfinite(msg.range_max) and msg.range_max > 0 else 10.0
        
        raw = np.array(msg.ranges, dtype=np.float32)
        invalid = (
            ~np.isfinite(raw)
            | (raw < msg.range_min)
            | (raw > max_r)
        )
        raw[invalid] = max_r
        raw = np.clip(raw, 0.0, max_r)
        
        # Interpolate if not exactly 360 points
        if len(raw) != 360:
            old_x = np.linspace(0.0, 1.0, len(raw))
            new_x = np.linspace(0.0, 1.0, 360)
            raw = np.interp(new_x, old_x, raw).astype(np.float32)
            
        # Normalise to [0, 1]
        normalised = raw / max_r
            
        # 2. Convert to PyTorch tensor
        # Shape: (1, 360)
        input_tensor = torch.tensor(normalised, dtype=torch.float32).unsqueeze(0)
        
        # 3. Model Inference
        with torch.no_grad():
            output = self.model(input_tensor)
            
        # output is shape (1, 2)
        linear_x = output[0, 0].item()
        angular_z = output[0, 1].item()
        
        # 4. Create and publish TwistStamped message
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'base_link'
        twist_msg.twist.linear.x = linear_x
        twist_msg.twist.angular.z = angular_z
        
        self.cmd_vel_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AIDriverNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("AI Driver Node stopped cleanly")
    except Exception as e:
        node.get_logger().error(f"Exception: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
