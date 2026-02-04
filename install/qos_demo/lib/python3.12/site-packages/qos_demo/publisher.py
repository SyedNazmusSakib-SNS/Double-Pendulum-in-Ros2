#!/usr/bin/env python3
"""
CONFIGURABLE PUBLISHER NODE
===========================
Demonstrates how to control:
- Publishing frequency (Hz)
- Queue size (buffer)
- QoS reliability settings
- Message size

Run with different frequencies:
    ros2 run qos_demo publisher --ros-args -p frequency:=10.0
    ros2 run qos_demo publisher --ros-args -p frequency:=100.0
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String, Float64
from rcl_interfaces.msg import ParameterDescriptor
import time


class ConfigurablePublisher(Node):
    """A publisher with configurable frequency and QoS settings."""

    def __init__(self):
        super().__init__('configurable_publisher')
        
        # =====================================================
        # DECLARE PARAMETERS (can be changed at runtime!)
        # =====================================================
        
        # Frequency parameter: How many messages per second
        self.declare_parameter(
            'frequency', 
            1.0,  # Default: 1 Hz
            ParameterDescriptor(description='Publishing frequency in Hz (1-100)')
        )
        
        # Queue size parameter: How many messages to buffer
        self.declare_parameter(
            'queue_size',
            10,  # Default: 10 messages
            ParameterDescriptor(description='Message queue/buffer size (1-100)')
        )
        
        # Reliability parameter: reliable or best_effort
        self.declare_parameter(
            'reliable',
            True,  # Default: reliable
            ParameterDescriptor(description='Use reliable QoS (True) or best_effort (False)')
        )
        
        # Message size parameter: simulate different payload sizes
        self.declare_parameter(
            'message_size',
            10,  # Default: 10 characters
            ParameterDescriptor(description='Size of message payload in characters')
        )
        
        # =====================================================
        # GET PARAMETER VALUES
        # =====================================================
        self.frequency = self.get_parameter('frequency').value
        self.queue_size = self.get_parameter('queue_size').value
        self.reliable = self.get_parameter('reliable').value
        self.message_size = self.get_parameter('message_size').value
        
        # =====================================================
        # CREATE QoS PROFILE
        # =====================================================
        qos_profile = QoSProfile(
            # RELIABILITY: Guarantee delivery or not
            reliability=ReliabilityPolicy.RELIABLE if self.reliable else ReliabilityPolicy.BEST_EFFORT,
            
            # HISTORY: Keep last N messages
            history=HistoryPolicy.KEEP_LAST,
            
            # DEPTH: Queue size (how many messages to keep)
            depth=self.queue_size,
            
            # DURABILITY: Keep messages for late subscribers?
            durability=DurabilityPolicy.VOLATILE
        )
        
        # =====================================================
        # CREATE PUBLISHERS
        # =====================================================
        
        # Main data publisher
        self.data_publisher = self.create_publisher(
            String, 
            'sensor_data', 
            qos_profile
        )
        
        # Statistics publisher (for monitoring)
        self.stats_publisher = self.create_publisher(
            String,
            'publisher_stats',
            10
        )
        
        # =====================================================
        # CREATE TIMER
        # =====================================================
        timer_period = 1.0 / self.frequency  # Convert Hz to seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # =====================================================
        # TRACKING VARIABLES
        # =====================================================
        self.message_count = 0
        self.start_time = time.time()
        self.last_stats_time = time.time()
        
        # =====================================================
        # LOG CONFIGURATION
        # =====================================================
        self.get_logger().info('=' * 60)
        self.get_logger().info('🚀 CONFIGURABLE PUBLISHER STARTED')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'📊 Configuration:')
        self.get_logger().info(f'   Frequency:    {self.frequency} Hz')
        self.get_logger().info(f'   Queue Size:   {self.queue_size}')
        self.get_logger().info(f'   Reliable:     {self.reliable}')
        self.get_logger().info(f'   Message Size: {self.message_size} chars')
        self.get_logger().info(f'   Period:       {timer_period*1000:.2f} ms')
        self.get_logger().info('=' * 60)
        self.get_logger().info('📤 Publishing to: /sensor_data')
        self.get_logger().info('📊 Stats on: /publisher_stats')
        self.get_logger().info('=' * 60)

    def timer_callback(self):
        """Called at the configured frequency to publish messages."""
        self.message_count += 1
        current_time = time.time()
        
        # Create message with configurable payload size
        payload = 'X' * self.message_size
        
        msg = String()
        msg.data = f'[{self.message_count:06d}] {payload} @ {current_time:.3f}'
        
        # Record publish time for latency calculation
        publish_time = time.time()
        
        # Publish the message
        self.data_publisher.publish(msg)
        
        # Log every message (for low frequencies) or every 10th (for high frequencies)
        if self.frequency <= 10 or self.message_count % 10 == 0:
            self.get_logger().info(f'📤 [{self.message_count}] Published at {self.frequency} Hz')
        
        # Publish statistics every 5 seconds
        if current_time - self.last_stats_time >= 5.0:
            self.publish_stats()
            self.last_stats_time = current_time

    def publish_stats(self):
        """Publish statistics about this publisher."""
        elapsed = time.time() - self.start_time
        actual_rate = self.message_count / elapsed if elapsed > 0 else 0
        
        stats_msg = String()
        stats_msg.data = (
            f'PUBLISHER STATS | '
            f'Messages: {self.message_count} | '
            f'Elapsed: {elapsed:.1f}s | '
            f'Target: {self.frequency} Hz | '
            f'Actual: {actual_rate:.2f} Hz | '
            f'Queue: {self.queue_size}'
        )
        self.stats_publisher.publish(stats_msg)
        
        self.get_logger().info('📊 ' + stats_msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = ConfigurablePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        elapsed = time.time() - node.start_time
        node.get_logger().info(f'👋 Shutting down. Sent {node.message_count} messages in {elapsed:.1f}s')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
