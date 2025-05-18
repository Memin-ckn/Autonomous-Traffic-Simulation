#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PointStamped
import math
import heapq

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        
        # Map representation
        self.grid = []
        self.grid_width = 0
        self.grid_height = 0
        self.start = None
        self.goal = None

        # ROS2 communication setup
        self.create_subscription(Int32MultiArray, '/map_data', self.map_callback, 10)
        self.create_subscription(PointStamped, '/start_point', self.start_callback, 10)
        self.create_subscription(PointStamped, '/goal_point', self.goal_callback, 10)
        self.path_pub = self.create_publisher(Path, '/global_path', 10)

    def map_callback(self, msg):
        # Convert flat map data to 2D grid
        flat_data = msg.data
        self.grid_height = msg.layout.dim[0].size
        self.grid_width = msg.layout.dim[1].size
        self.grid = [
            flat_data[i * self.grid_width:(i + 1) * self.grid_width]
            for i in range(self.grid_height)
        ]
        self.get_logger().info("Map updated")
        self.plan_path()

    def start_callback(self, msg):
        # Store start coordinates
        self.start = (int(msg.point.x), int(msg.point.y))
        self.get_logger().info(f"Start point set: {self.start}")
        self.plan_path()

    def goal_callback(self, msg):
        # Store goal coordinates
        self.goal = (int(msg.point.x), int(msg.point.y))
        self.get_logger().info(f"Goal point set: {self.goal}")
        self.plan_path()

    def plan_path(self):
        # A* path planning implementation
        if self.start is None or self.goal is None or not self.grid:
            return

        def heuristic(a, b):
            # Manhattan distance heuristic
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        close_set = set()
        came_from = {}
        gscore = {self.start: 0}
        fscore = {self.start: heuristic(self.start, self.goal)}
        oheap = []
        heapq.heappush(oheap, (fscore[self.start], self.start))
        
        while oheap:
            current = heapq.heappop(oheap)[1]

            if current == self.goal:
                # Reconstruct and publish path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(self.start)
                path.reverse()
                self.publish_path(path)
                return

            close_set.add(current)
            for dx, dy in neighbors:
                neighbor = current[0] + dx, current[1] + dy

                # Skip invalid or blocked neighbors
                if not (0 <= neighbor[0] < self.grid_height and 0 <= neighbor[1] < self.grid_width):
                    continue
                if self.grid[neighbor[0]][neighbor[1]] == 1:
                    continue
                if neighbor in close_set:
                    continue

                # Update path if better route found
                tentative_g_score = gscore[current] + 1
                if tentative_g_score < gscore.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + heuristic(neighbor, self.goal)
                    heapq.heappush(oheap, (fscore[neighbor], neighbor))

        self.get_logger().warn("No path found!")

    def publish_path(self, path):
        # Convert path to ROS2 Path message
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        for x, y in path:
            pose = PoseStamped()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Published path with {len(path)} points")

def main(args=None):
    rclpy.init(args=args)
    planner = PathPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
