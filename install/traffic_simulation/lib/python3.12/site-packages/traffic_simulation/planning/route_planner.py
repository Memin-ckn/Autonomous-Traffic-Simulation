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
        self.origin = "37.7749,-122.4194"  #(San Francisco)
        self.route_data = []  # saving road's
        self.publisher_ = self.create_publisher(PoseStamped, 'goal', 10)

        # get user target map
        self.destination = self.get_user_destination()
        if self.destination:
            self.get_logger().info(f"Hedef adres: {self.destination}")
            self.fetch_route()
        else:
            self.get_logger().warning("Hedef belirlenmedi. Çıkılıyor.")
            rclpy.shutdown()

    def get_user_destination(self):
        """ GUI."""
        root = Tk()
        root.withdraw()
        destination = askstring("Hedef Belirleme", "Lütfen hedef adresini girin:")
        root.destroy()
        return destination

    def fetch_route(self):
        """Google Maps Directions API catching datas."""
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
                self.get_logger().info("Yol başarıyla alındı.")
                self.publish_route()
            else:
                self.get_logger().error(f"Google Maps API hatası: {response['status']}")
        except Exception as e:
            self.get_logger().error(f"Rota alınırken hata oluştu: {e}")

    def publish_route(self):
        """publish on "goal" topic"""
        for idx, (start, end) in enumerate(self.route_data):
            goal_msg = PoseStamped()
            goal_msg.pose.position.x = end["lat"]
            goal_msg.pose.position.y = end["lng"]
            self.publisher_.publish(goal_msg)
            self.get_logger().info(f"{idx + 1}. Hedef yayınlandı: {end['lat']}, {end['lng']}")


def main(args=None):
    rclpy.init(args=args)
    node = RoutePlanner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
