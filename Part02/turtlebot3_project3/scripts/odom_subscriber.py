#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('odom_subscriber')
        # Create a subscription to the "odom" topic with a queue size of 10.
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.listener_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg: Odometry):
        # Log the received odometry data.
        # You can access position and orientation via msg.pose.pose
        # and velocity via msg.twist.twist.
        self.get_logger().info(
            f"Received odometry: Position -> x: {msg.pose.pose.position.x:.2f}, "
            f"y: {msg.pose.pose.position.y:.2f}, z: {msg.pose.pose.position.z:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = OdomSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
