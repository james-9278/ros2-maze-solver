#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time

class MazeSolverNode(Node):
    def __init__(self):
        super().__init__("maze_solver_node")
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription_ = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        
        # --- State Machine ---
        self.state_ = 'finding_wall'
        self.get_logger().info('Maze Solver Node started. Initial state: finding_wall')

        # --- Parameters ---
        self.forward_speed_ = 0.12
        self.turning_speed_ = 0.6
        self.wall_dist_ = 0.35
        self.front_clear_dist_ = 0.5 # Must be larger than wall_dist_
        self.side_clear_dist_ = 0.5

    def stop_robot(self):
        """Publishes a zero velocity Twist message to stop the robot."""
        self.publisher_.publish(Twist())

    def change_state(self, new_state):
        if self.state_ != new_state:
            self.get_logger().info(f'Changing state from {self.state_} to {new_state}')
            self.state_ = new_state
            # It's good practice to stop the robot when changing state
            self.stop_robot()
            time.sleep(0.1) # Brief pause

    def scan_callback(self, msg):
        # Process sensor data to avoid errors with 'inf' and 'nan'
        ranges = [r if (not (r is None or r == float('inf') or r == float('nan'))) else 10.0 for r in msg.ranges]
        
        # Define sensor regions
        front_dist = min(ranges[0:15] + ranges[345:360])
        left_dist = min(ranges[85:95])
        right_dist = min(ranges[265:275])
        
        command = Twist()

        # --- State Machine Logic ---
        if self.state_ == 'finding_wall':
            if front_dist < self.wall_dist_:
                self.change_state('turning_left')
            else:
                command.linear.x = self.forward_speed_
                
        elif self.state_ == 'turning_left':
            # Turn left until the front is clear
            if front_dist > self.front_clear_dist_:
                self.change_state('following_wall')
            else:
                command.angular.z = self.turning_speed_
                
        elif self.state_ == 'following_wall':
            # If wall is now in front, we've hit a corner -> turn left
            if front_dist < self.wall_dist_:
                self.change_state('turning_left')
            # If there's no wall on the right (e.g. an opening), turn right into it
            elif right_dist > self.side_clear_dist_:
                # This makes it turn into open hallways on the right
                command.linear.x = self.forward_speed_ * 0.5
                command.angular.z = -self.turning_speed_ * 0.75
            else:
                # Follow the wall using a proportional controller
                error = self.wall_dist_ - right_dist
                command.linear.x = self.forward_speed_
                # The angular velocity is proportional to the error
                command.angular.z = error * 1.5 
        
        else:
            self.get_logger().error(f"Unknown state: {self.state_}")

        self.publisher_.publish(command)

def main(args=None):
    rclpy.init(args=args)
    node = MazeSolverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT). Shutting down...')
    finally:
        # Before shutting down, make sure to stop the robot
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown
