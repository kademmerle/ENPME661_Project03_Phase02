#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
import csv
import os

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory


class Controller(Node):
    def __init__(self):
        super().__init__('Controller')
        pkg_share = get_package_share_directory('turtlebot3_project3')
        csv_path = os.path.join(pkg_share, 'scripts', 'waypoints.csv')
        self.waypoints = []


        # ---------- ROS Interfaces ----------
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ---------- Parameters ----------
        # List of waypoints (x, y), in meters
        r = 0.033
        with open(csv_path, 'r') as file:
            reader = csv.reader(file)
            w_l, w_r = next(reader)
            w_l = float(w_l) * 2 * math.pi / 60 
            w_r = float(w_r) * 2 * math.pi / 60
            v_l = w_l*r
            v_r = w_r*r
            for row in reader:
                x_str, y_str = row
                x = float(x_str) / 100
                y = (float(y_str) ) /100 
                if y > 1.15:
                    y = 1.15
                self.waypoints.append((x, y))
                max_linear_velocity = (float(v_l) + float(v_r)) / 4 # (manual damen)
                self.max_angular_velocity = max(
                    ((float(w_r) - float(w_l)) / 2),
                    ((float(w_l) - float(w_r)) / 2),
                    float(w_l),
                    float(w_r)
                ) / 2
                self.get_logger().info(f"Max linear velocity: {max_linear_velocity}")
                self.get_logger().info(f"Max angular velocity: {self.max_angular_velocity}")

        # self.waypoints = [
        #     (0.7, 0.8),
        #     (1.6, 0.8),
        #     (1.6, -0.72),
        #     (2.7, -0.72),
        #     (2.7, 0.0),
        #     (3.7, 0.0),
        #     (3.7, 0.8),
        #     (5.0, 0.8),
        #     (5.0, .3),
        #     (5.5, .3),

        # ]
        self.current_waypoint_idx = 0
        self.distance_threshold = 0.05  # If within this move to the next waypoint
        
        # Base PID gains (we adapt Kp based on distance to path)
        self.base_kp = 1.0
        self.ki      = 0.1
        self.kd      = 0.1


        # PID state
        self.integral_error = 0.1
        self.prev_error     = 0.1

        # we'll just control angular.z to turn toward waypoint. For linear velocity, we use constant.
        self.forward_speed = max_linear_velocity  # m/s

        # Timer for control loop
        self.control_rate = 10.0  # Hz
        self.dt           = 1.0 / self.control_rate
        self.timer        = self.create_timer(self.dt, self.control_loop)

        # Robot pose from odometry
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0

        self.get_logger().info("Controller node started.")

    def odom_callback(self, msg):
        """
        Extract robot's (x, y, yaw) from odometry message
        """
        # Position
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Orientation in quaternion form
        quat = msg.pose.pose.orientation
        # Convert quaternion to Euler yaw (assuming no pitch/roll)
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        self.yaw   = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        """
        Periodically called function that:
        1) Pick current waypoint
        2) Compute heading error
        3) Adapt Kp based on distance to waypoint and apply modest integral and derivative gains
        4) Publish updated control command
        """

        # Check if we finished all waypoints
        if self.current_waypoint_idx >= len(self.waypoints):
            self.stop_robot()
            self.get_logger().info("All waypoints reached. Stopping.")
            return

        # Current target waypoint
        wx, wy = self.waypoints[self.current_waypoint_idx]

        # Distance to current waypoint
        dx = wx - self.x
        dy = wy - self.y
        dist_error = math.hypot(dx, dy)

        # If close to the current waypoint, move on to the next
        if dist_error < self.distance_threshold:
            self.current_waypoint_idx += 1
            self.get_logger().info(f"Reached waypoint {self.current_waypoint_idx}.")
            return

        # Compute heading to waypoint
        desired_yaw = math.atan2(dy, dx)

        # Heading error
        heading_error = self.normalize_angle(desired_yaw - self.yaw)

        # 3) Adapt Kp based on how far we are from the path
        gain_factor = 1.0 + min(dist_error, 1.0)  # cap at 1 meter for safety
        kp_adapted = self.base_kp * gain_factor

        # PID on heading error w/ adaptive P Gain
        self.integral_error += heading_error * self.dt
        deriv_error = (heading_error - self.prev_error) / self.dt

        control_angular = (kp_adapted * heading_error
                           + self.ki * self.integral_error
                           + self.kd * deriv_error)

        # Constuct and publish the command message
        cmd_msg = Twist()
        cmd_msg.linear.x = self.forward_speed

        # Limit angular velocity to max_angular_velocity
        if abs(control_angular) > abs(self.max_angular_velocity):
            control_angular = math.copysign(self.max_angular_velocity, control_angular)
        cmd_msg.angular.z = control_angular
        self.cmd_pub.publish(cmd_msg)

        # Store current error for next iteration
        self.prev_error = heading_error

        # Log
        self.get_logger().info(
            f"Waypoint: ({wx:.2f}, {wy:.2f}), Pos: ({self.x:.2f}, {self.y:.2f}), "
            f"DistErr: {dist_error:.2f}, HeadingErr: {heading_error:.2f}, "
            f"Kp: {kp_adapted:.2f}, AngZ: {control_angular:.2f}"
        )

    def stop_robot(self):
        """
        Publish zero velocity to stop the robot.
        """
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def normalize_angle(self, angle):
        """
        Normalize the angle to [-pi, pi].
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
