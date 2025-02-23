#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class WaypointNavigation(Node):
    def __init__(self):
        super().__init__('waypoint_navigation')

        # Declare parameters and get values
        self.declare_parameter('waypoint_1_x', 2.0)
        self.declare_parameter('waypoint_1_y', 0.0)
        self.declare_parameter('waypoint_2_x', 4.0)
        self.declare_parameter('waypoint_2_y', 0.0)
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)

        self.waypoints = [
            (
                self.get_parameter('waypoint_1_x').value,
                self.get_parameter('waypoint_1_y').value
            ),
            (
                self.get_parameter('waypoint_2_x').value,
                self.get_parameter('waypoint_2_y').value
            )
        ]

        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value

        self.current_waypoint_index = 0
        self.odom_sub = self.create_subscription(
            Odometry, '/diff_cont/odom', self.odom_callback, 10)

        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)

        # PID-related tracking
        self.previous_error_lin = 0.0
        self.integral_lin = 0.0

        self.previous_error_ang = 0.0
        self.integral_ang = 0.0

        self.get_logger().info("Waypoint Navigation node started.")

    def odom_callback(self, msg):
        # Extract the robot's current position
        position = msg.pose.pose.position
        x_current = position.x
        y_current = position.y

        # Current orientation (yaw) from quaternion
        orientation_q = msg.pose.pose.orientation
        yaw = self.quaternion_to_euler(orientation_q)

        # If we reached all waypoints, stop
        if self.current_waypoint_index >= len(self.waypoints):
            self.stop_robot()
            return

        # Target waypoint
        x_goal, y_goal = self.waypoints[self.current_waypoint_index]
        distance = math.sqrt((x_goal - x_current)**2 + (y_goal - y_current)**2)

        # Threshold to switch to the next waypoint
        if distance < 0.2:
            self.get_logger().info(f"Reached waypoint {self.current_waypoint_index+1}")
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.waypoints):
                self.stop_robot()
                return

        # Compute heading error
        angle_to_goal = math.atan2(y_goal - y_current, x_goal - x_current)
        angle_error = angle_to_goal - yaw

        # Normalize angle_error to (-pi, pi)
        if angle_error > math.pi:
            angle_error -= 2.0 * math.pi
        elif angle_error < -math.pi:
            angle_error += 2.0 * math.pi

        # PID for linear velocity
        # The "error" is simply the distance
        error_lin = distance
        self.integral_lin += error_lin
        derivative_lin = error_lin - self.previous_error_lin
        linear_output = (self.kp * error_lin) + (self.ki * self.integral_lin) + (self.kd * derivative_lin)
        self.previous_error_lin = error_lin

        # PID for angular velocity
        error_ang = angle_error
        self.integral_ang += error_ang
        derivative_ang = error_ang - self.previous_error_ang
        angular_output = (self.kp * error_ang) + (self.ki * self.integral_ang) + (self.kd * derivative_ang)
        self.previous_error_ang = error_ang

        # Publish the velocities
        twist_msg = Twist()
        twist_msg.linear.x = linear_output
        twist_msg.angular.z = angular_output
        self.cmd_vel_pub.publish(twist_msg)

    def stop_robot(self):
        """Stop the robot completely"""
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)

    def quaternion_to_euler(self, orientation_q):
        """Convert quaternion to yaw (euler)"""
        import math
        x = orientation_q.x
        y = orientation_q.y
        z = orientation_q.z
        w = orientation_q.w

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigation()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
