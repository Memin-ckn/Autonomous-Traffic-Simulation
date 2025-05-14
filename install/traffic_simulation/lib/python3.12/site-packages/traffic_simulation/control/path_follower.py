#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
import math

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        self.subscription = self.create_subscription(
            Path,
            'global_path',
            self.path_callback,
            10)
        self.publisher = self.create_publisher(Twist, 'raw_cmd_vel', 10)
        self.current_path = []
        self.current_goal_index = 0
        self.linear_speed = 0.5
        self.angular_speed = 0.8

    def path_callback(self, msg):
        self.current_path = [(pose.pose.position.x, pose.pose.position.y) 
                           for pose in msg.poses]
        self.current_goal_index = 0
        self.get_logger().info(f"New path received: {len(self.current_path)} points")

    def calculate_velocity(self, current_pose):
        if not self.current_path or self.current_goal_index >= len(self.current_path):
            return Twist()
        
        goal = self.current_path[self.current_goal_index]
        dx = goal[0] - current_pose.x
        dy = goal[1] - current_pose.y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < 0.1:  # Reached goal
            self.current_goal_index += 1
            return self.calculate_velocity(current_pose)
        
        twist = Twist()
        twist.linear.x = min(self.linear_speed, distance*0.5)
        angle_to_goal = math.atan2(dy, dx)
        twist.angular.z = self.angular_speed * angle_to_goal
        
        return twist

def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()