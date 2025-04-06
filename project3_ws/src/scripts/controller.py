#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        # Create publisher for Twist messages on /cmd_vel topic with a queue size of 10
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        timer_period = 0.1 # Publish at .1 Hz (once per second)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        # Set forward speed (m/s)
        msg.linear.x = 0.2
        # Set turning speed (rad/s)
        msg.angular.z = 0.3
        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Published cmd_vel: linear.x={msg.linear.x}, angular.z={msg.angular.z}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()