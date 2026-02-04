#!/usr/bin/env python3
"""
NUMBER SUBSCRIBER NODE
======================
This node subscribes to two number topics and calculates:
- Sum (A + B)
- Difference (A - B)
- Product (A × B)

Topics Subscribed:
    /number_a (std_msgs/Float64): First number
    /number_b (std_msgs/Float64): Second number

Topics Published:
    /result (std_msgs/Float64): The sum of A + B
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class NumberSubscriber(Node):
    """A subscriber node that receives numbers and calculates their sum."""

    def __init__(self):
        # Initialize the node with name 'number_subscriber'
        super().__init__('number_subscriber')
        
        # Store the latest received values
        self.number_a = None
        self.number_b = None
        
        # Create subscribers for both number topics
        # Subscriber(message_type, topic_name, callback, queue_size)
        self.sub_a = self.create_subscription(
            Float64,
            'number_a',
            self.callback_a,
            10
        )
        
        self.sub_b = self.create_subscription(
            Float64,
            'number_b',
            self.callback_b,
            10
        )
        
        # Create a publisher for the result
        self.publisher_result = self.create_publisher(Float64, 'result', 10)
        
        self.get_logger().info('🎯 Number Subscriber Node Started!')
        self.get_logger().info('📥 Subscribed to: /number_a and /number_b')
        self.get_logger().info('📤 Publishing to: /result')

    def callback_a(self, msg):
        """Called when a message is received on /number_a."""
        self.number_a = msg.data
        self.calculate_and_publish()

    def callback_b(self, msg):
        """Called when a message is received on /number_b."""
        self.number_b = msg.data
        self.calculate_and_publish()

    def calculate_and_publish(self):
        """Calculate sum and publish when both numbers are available."""
        # Only calculate if we have both numbers
        if self.number_a is None or self.number_b is None:
            return
        
        # Calculate results
        sum_result = self.number_a + self.number_b
        diff_result = self.number_a - self.number_b
        product_result = self.number_a * self.number_b
        
        # Publish the sum
        result_msg = Float64()
        result_msg.data = sum_result
        self.publisher_result.publish(result_msg)
        
        # Log the calculations
        self.get_logger().info('━' * 50)
        self.get_logger().info(f'📊 Received Numbers:')
        self.get_logger().info(f'   A = {self.number_a:.2f}')
        self.get_logger().info(f'   B = {self.number_b:.2f}')
        self.get_logger().info(f'📈 Results:')
        self.get_logger().info(f'   Sum (A+B)     = {sum_result:.2f}')
        self.get_logger().info(f'   Diff (A-B)    = {diff_result:.2f}')
        self.get_logger().info(f'   Product (A×B) = {product_result:.2f}')
        
        # Reset to wait for next pair
        self.number_a = None
        self.number_b = None


def main(args=None):
    # Initialize ROS 2
    rclpy.init(args=args)
    
    # Create our node
    node = NumberSubscriber()
    
    try:
        # Keep the node running
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        node.get_logger().info('👋 Shutting down Number Subscriber...')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
