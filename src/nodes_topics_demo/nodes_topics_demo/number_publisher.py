#!/usr/bin/env python3
"""
NUMBER PUBLISHER NODE
=====================
This node publishes two random numbers to separate topics.

Topics Published:
    /number_a (std_msgs/Float64): First random number
    /number_b (std_msgs/Float64): Second random number

The numbers are published every 1 second.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random


class NumberPublisher(Node):
    """A simple publisher node that sends two random numbers."""

    def __init__(self):
        # Initialize the node with name 'number_publisher'
        super().__init__('number_publisher')
        
        # Create two publishers for our two numbers
        # Publisher(message_type, topic_name, queue_size)
        self.publisher_a = self.create_publisher(Float64, 'number_a', 10)
        self.publisher_b = self.create_publisher(Float64, 'number_b', 10)
        
        # Create a timer that calls our callback every 1 second
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Counter to track how many messages we've sent
        self.count = 0
        
        self.get_logger().info('🚀 Number Publisher Node Started!')
        self.get_logger().info('📤 Publishing to: /number_a and /number_b')

    def timer_callback(self):
        """Called every second to publish new random numbers."""
        # Generate two random numbers between 1 and 100
        num_a = random.uniform(1.0, 100.0)
        num_b = random.uniform(1.0, 100.0)
        
        # Create message objects
        msg_a = Float64()
        msg_a.data = num_a
        
        msg_b = Float64()
        msg_b.data = num_b
        
        # Publish the messages
        self.publisher_a.publish(msg_a)
        self.publisher_b.publish(msg_b)
        
        self.count += 1
        
        # Log what we published
        self.get_logger().info(
            f'[{self.count}] Published: A={num_a:.2f}, B={num_b:.2f}'
        )


def main(args=None):
    # Initialize ROS 2
    rclpy.init(args=args)
    
    # Create our node
    node = NumberPublisher()
    
    try:
        # Keep the node running until interrupted
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        node.get_logger().info('👋 Shutting down Number Publisher...')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
