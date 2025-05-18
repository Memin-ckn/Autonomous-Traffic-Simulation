#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import requests
from tkinter import Tk
from tkinter.simpledialog import askstring


class RoutePlanner(Node):
    def __init__(self):
        super().__init__('route_planner')
        self.api_key = "YOUR_GOOGLE_MAPS_API_KEY"
        self.origin = "37.7749,-122.4194"  # San Francisco coordinates
        self.route_data = []
        self.publisher_ = self.create_publisher(PoseStamped, 'goal', 10)

        # Get destination from user and start planning
        self.destination = self.get_user_destination()
        if self.destination:
            self.get_logger().info(f"Target address: {self.destination}")
            self.fetch_route()
        else:
            self.get_logger().warning("No destination set. Exiting.")
            rclpy.shutdown()

    def get_user_destination(self):
        # Prompt user for destination via GUI
        root = Tk()
        root.withdraw()
        destination = askstring("Set Destination", "Enter destination address:")
        root.destroy()
        return destination

    def fetch_route(self):
        # Get route data from Google Maps API
        directions_url = "https://maps.googleapis.com/maps/api/directions/json"
        params = {
            "origin": self.origin,
            "destination": self.destination,
            "key": self.api_key
        }

        try:
            response = requests.get(directions_url, params=params).json()
            if response["status"] == "OK":
                steps = response["routes"][0]["legs"][0]["steps"]
                for step in steps:
                    start = step["start_location"]
                    end = step["end_location"]
                    self.route_data.append((start, end))
                self.get_logger().info("Route fetched successfully")
                self.publish_route()
            else:
                self.get_logger().error(f"Google Maps API error: {response['status']}")
        except Exception as e:
            self.get_logger().error(f"Error fetching route: {e}")

    def publish_route(self):
        # Publish waypoints to goal topic
        for idx, (start, end) in enumerate(self.route_data):
            goal_msg = PoseStamped()
            goal_msg.pose.position.x = end["lat"]
            goal_msg.pose.position.y = end["lng"]
            self.publisher_.publish(goal_msg)
            self.get_logger().info(f"Published goal {idx + 1}: {end['lat']}, {end['lng']}")


def main(args=None):
    rclpy.init(args=args)
    node = RoutePlanner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
