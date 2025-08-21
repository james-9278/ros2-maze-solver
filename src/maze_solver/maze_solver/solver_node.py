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
        
        # State
        self.state_ = 'finding_wall'
        self.get_logger().info('Maze Solver Node started. Initial state: finding_wall')

        # Params
        self.forward_speed_ = 0.12 # has to be slow to avoid crashing 
        self.turning_speed_ = 0.6
        self.wall_dist_ = 0.35 # any closer and it crashes
        self.front_clear_dist_ = 0.5 # Must be larger than wall_dist_
        self.side_clear_dist_ = 0.5

    def stop_robot(self):
        # Stops the robot using a 0 velocity twist message
        self.publisher_.publish(Twist())

    def change_state(self, new_state):
        if self.state_ != new_state:
            self.get_logger().info(f'Changing state from {self.state_} to {new_state}')
            self.state_ = new_state
            # Stop when changing state so the forward momentum doesnt keep going and crash
            self.stop_robot()
            time.sleep(0.1) # stop for .1

    def scan_callback(self, msg):
        # Process sensor data to avoid errors with 'inf' and 'nan'
        ranges = [r if (not (r is None or r == float('inf') or r == float('nan'))) else 10.0 for r in msg.ranges]
        
        # Define sensor regions
        front_dist = min(ranges[0:15] + ranges[345:360]) # gets the 30 degrees infront of the robot
        left_dist = min(ranges[85:95]) # 10 degree arc on the left
        right_dist = min(ranges[265:275]) # 10 degree arc on the right
        # had to check left & right bc it would only see things directly infront and clip left or right
        
        command = Twist()

        # State machine logic
        
        # wall finding
        # since i am following the right wall, when i see the right wall, turn left
        if self.state_ == 'finding_wall':
            if front_dist < self.wall_dist_:
                self.change_state('turning_left')
            else:
                command.linear.x = self.forward_speed_
        
        # left turn
        # turn left until the 30 deg arc in front is clear         
        elif self.state_ == 'turning_left':
            
            if front_dist > self.front_clear_dist_:
                self.change_state('following_wall')
            else:
                command.angular.z = self.turning_speed_
        
        # found wall to follow
        elif self.state_ == 'following_wall':
        
            # If the wall is in front, this means there is a corner so we turn left
           
            if front_dist < self.wall_dist_:
                self.change_state('turning_left')
                
            # If there is no wall on the right, this means there is an opening so we need to turn right into it
            
            elif right_dist > self.side_clear_dist_:
            
                command.linear.x = self.forward_speed_ * 0.5
                command.angular.z = -self.turning_speed_ * 0.75
                
            else:
            
                # Follow the wall using a proportional controller
                # we calculate the difference from how far he wants to be away from the wall vs how actually far he is away
                error = self.wall_dist_ - right_dist
                command.linear.x = self.forward_speed_
                
                # Steers based on the error 
                # if the robot is too far away from the wall, the error will be positive, 
                #it will turn right as the angular velocity is negative
                # vice versa for too close - smart 
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
        node.get_logger().info('Keyboard Interrupt. Shutting down.')
    finally:
        # Before shutting down, make sure to stop the robot
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown
