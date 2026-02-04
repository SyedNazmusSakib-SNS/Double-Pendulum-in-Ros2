#!/usr/bin/env python3
"""
CONFIGURABLE SUBSCRIBER NODE
============================
Demonstrates how to:
- Match QoS settings with publisher
- Monitor message latency
- Track dropped messages
- Handle different reliability modes

Run with matching QoS:
    ros2 run qos_demo subscriber --ros-args -p reliable:=true
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor
import time
import re


class ConfigurableSubscriber(Node):
    """A subscriber that monitors message reception and calculates statistics."""

    def __init__(self):
        super().__init__('configurable_subscriber')
        
        # =====================================================
        # DECLARE PARAMETERS
        # =====================================================
        
        self.declare_parameter(
            'queue_size',
            10,
            ParameterDescriptor(description='Message queue/buffer size')
        )
        
        self.declare_parameter(
            'reliable',
            True,
            ParameterDescriptor(description='Use reliable QoS (must match publisher!)')
        )
        
        self.declare_parameter(
            'simulate_slow',
            0.0,
            ParameterDescriptor(description='Simulate slow processing (seconds per message)')
        )
        
        # =====================================================
        # GET PARAMETER VALUES
        # =====================================================
        self.queue_size = self.get_parameter('queue_size').value
        self.reliable = self.get_parameter('reliable').value
        self.simulate_slow = self.get_parameter('simulate_slow').value
        
        # =====================================================
        # CREATE QoS PROFILE (must be compatible with publisher!)
        # =====================================================
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE if self.reliable else ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=self.queue_size,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # =====================================================
        # CREATE SUBSCRIBER
        # =====================================================
        self.subscription = self.create_subscription(
            String,
            'sensor_data',
            self.listener_callback,
            qos_profile
        )
        
        # =====================================================
        # STATISTICS TRACKING
        # =====================================================
        self.message_count = 0
        self.last_sequence = 0
        self.dropped_count = 0
        self.start_time = time.time()
        self.latencies = []
        self.last_stats_time = time.time()
        
        # =====================================================
        # LOG CONFIGURATION
        # =====================================================
        self.get_logger().info('=' * 60)
        self.get_logger().info('🎯 CONFIGURABLE SUBSCRIBER STARTED')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'📊 Configuration:')
        self.get_logger().info(f'   Queue Size:    {self.queue_size}')
        self.get_logger().info(f'   Reliable:      {self.reliable}')
        self.get_logger().info(f'   Slow Process:  {self.simulate_slow}s')
        self.get_logger().info('=' * 60)
        self.get_logger().info('📥 Subscribed to: /sensor_data')
        self.get_logger().info('=' * 60)

    def listener_callback(self, msg):
        """Called when a message is received."""
        receive_time = time.time()
        self.message_count += 1
        
        # =====================================================
        # PARSE MESSAGE TO GET SEQUENCE NUMBER AND TIMESTAMP
        # =====================================================
        try:
            # Extract sequence number from message: [000001] XXX @ 1234.567
            match = re.search(r'\[(\d+)\].*@ ([\d.]+)', msg.data)
            if match:
                sequence = int(match.group(1))
                send_time = float(match.group(2))
                
                # Calculate latency
                latency = (receive_time - send_time) * 1000  # Convert to ms
                self.latencies.append(latency)
                
                # Keep only last 100 latencies for averaging
                if len(self.latencies) > 100:
                    self.latencies.pop(0)
                
                # Check for dropped messages
                if self.last_sequence > 0 and sequence > self.last_sequence + 1:
                    dropped = sequence - self.last_sequence - 1
                    self.dropped_count += dropped
                    self.get_logger().warn(f'⚠️ DROPPED {dropped} messages!')
                
                self.last_sequence = sequence
                
        except Exception as e:
            self.get_logger().error(f'Parse error: {e}')
        
        # =====================================================
        # SIMULATE SLOW PROCESSING (to demonstrate queue behavior)
        # =====================================================
        if self.simulate_slow > 0:
            time.sleep(self.simulate_slow)
        
        # =====================================================
        # LOG RECEIVED MESSAGE
        # =====================================================
        if self.message_count % 10 == 0 or self.message_count <= 5:
            avg_latency = sum(self.latencies) / len(self.latencies) if self.latencies else 0
            self.get_logger().info(
                f'📥 [{self.message_count}] Received | '
                f'Latency: {latency:.2f}ms | '
                f'Avg: {avg_latency:.2f}ms'
            )
        
        # =====================================================
        # PRINT DETAILED STATS EVERY 5 SECONDS
        # =====================================================
        if receive_time - self.last_stats_time >= 5.0:
            self.print_stats()
            self.last_stats_time = receive_time

    def print_stats(self):
        """Print detailed reception statistics."""
        elapsed = time.time() - self.start_time
        rate = self.message_count / elapsed if elapsed > 0 else 0
        avg_latency = sum(self.latencies) / len(self.latencies) if self.latencies else 0
        min_latency = min(self.latencies) if self.latencies else 0
        max_latency = max(self.latencies) if self.latencies else 0
        
        self.get_logger().info('━' * 60)
        self.get_logger().info('📊 SUBSCRIBER STATISTICS')
        self.get_logger().info('━' * 60)
        self.get_logger().info(f'   Messages Received: {self.message_count}')
        self.get_logger().info(f'   Messages Dropped:  {self.dropped_count}')
        self.get_logger().info(f'   Receive Rate:      {rate:.2f} Hz')
        self.get_logger().info(f'   Elapsed Time:      {elapsed:.1f} s')
        self.get_logger().info('━' * 60)
        self.get_logger().info(f'   ⏱️ LATENCY STATS (last 100 msgs):')
        self.get_logger().info(f'      Min:     {min_latency:.2f} ms')
        self.get_logger().info(f'      Max:     {max_latency:.2f} ms')
        self.get_logger().info(f'      Average: {avg_latency:.2f} ms')
        self.get_logger().info('━' * 60)


def main(args=None):
    rclpy.init(args=args)
    node = ConfigurableSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.print_stats()
        node.get_logger().info(f'👋 Shutting down. Received {node.message_count} messages.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
