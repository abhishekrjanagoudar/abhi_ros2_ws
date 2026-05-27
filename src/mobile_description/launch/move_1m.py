#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class Move1m(Node):
    def __init__(self):
        super().__init__('move_1m_node')
        # Use sim time since we're in Gazebo
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        self.pub = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        self.sub = self.create_subscription(Odometry, '/diff_cont/odom', self.odom_cb, 10)
        
        self.start_x = None
        self.start_y = None
        self.distance_moved = 0.0
        self.target_distance = 1.0  # meters
        
        self.timer = self.create_timer(0.1, self.cmd_vel_loop)
        self.get_logger().info("Moving forward 1 meter...")

    def odom_cb(self, msg):
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        
        if self.start_x is None:
            self.start_x = current_x
            self.start_y = current_y
            
        self.distance_moved = math.sqrt(
            (current_x - self.start_x)**2 + 
            (current_y - self.start_y)**2
        )

    def cmd_vel_loop(self):
        msg = Twist()
        
        if self.start_x is None:
            return # waiting for odom

        if self.distance_moved < self.target_distance:
            msg.linear.x = 0.2  # m/s
            self.pub.publish(msg)
        else:
            msg.linear.x = 0.0
            self.pub.publish(msg)
            self.get_logger().info("Reached 1 meter. Stopping.")
            rclpy.shutdown()

def main():
    rclpy.init()
    node = Move1m()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
