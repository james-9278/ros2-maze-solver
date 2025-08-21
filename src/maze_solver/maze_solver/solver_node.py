#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time # <-- 1. IMPORT TIME

class MazeSolverNode(Node):
    def __init__(self):
        super().__init__("maze_solver_node")
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription_ = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        
        self.state_ = 'finding_wall'
        self.state_entry_time_ = 0.0 
        self.get_logger().info('Maze Solver Node started. Initial state: finding_wall')

        # Params
        self.forward_speed_ = 0.12 # any more and likely to fall
        self.turning_speed_ = 0.6
        self.wall_dist_ = 0.35
        self.front_clear_dist_ = 0.5
        self.side_clear_dist_ = 0.5
        self.clear_trap_duration_ = 2.0 # time to drive forward after a U-turn

    def stop_robot(self):
        self.publisher_.publish(Twist())

    def change_state(self, new_state):
        if self.state_ != new_state:
            self.get_logger().info(f'Changing state from {self.state_} to {new_state}')
            self.state_ = new_state
            self.state_entry_time_ = time.time() # <-- record time on state change
            self.stop_robot()
            time.sleep(0.1)

    def scan_callback(self, msg):
        # Process sensor data
        ranges = [r if (not (r is None or r == float('inf') or r == float('nan'))) else 10.0 for r in msg.ranges]
        
        front_dist = min(ranges[0:30] + ranges[330:360]) # bigger range of 60 degrees to see close
        right_dist = min(ranges[265:275])
        
        command = Twist()

        # if wall is head turn left
        if self.state_ == 'finding_wall':
            if front_dist < self.wall_dist_ + 0.1:
                self.change_state('turning_left')
            else:
                command.linear.x = self.forward_speed_
	# when no wall is ahead follow right wall
        elif self.state_ == 'turning_left':
            if front_dist > self.front_clear_dist_:
                self.change_state('following_wall')
            else:
                command.angular.z = self.turning_speed_

        # if a corner is ahead, stop forward motion and pivot left
        elif self.state_ == 'following_wall':
      
            if front_dist < self.wall_dist_:
              
                command.linear.x = 0.0
                command.angular.z = self.turning_speed_
            
            # when the right wall opens up, do a slow right turn
            elif right_dist > self.side_clear_dist_:
               
                command.linear.x = self.forward_speed_ * 0.5 # Slow down
                command.angular.z = -self.turning_speed_ * 0.7 # Gentle turn
            
            # normal following
            else:
                # drive forward while adjusting to the right wall
                command.linear.x = self.forward_speed_
                
                # use the proportional controller for smooth adjustments
                error = self.wall_dist_ - right_dist
                turn_adjustment = -error * 2.5 # Gain of 2.5
                
                # the turning speed is capped to prevent over correction
                max_turn = self.turning_speed_ * 0.5
                command.angular.z = max(-max_turn, min(turn_adjustment, max_turn))

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
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
