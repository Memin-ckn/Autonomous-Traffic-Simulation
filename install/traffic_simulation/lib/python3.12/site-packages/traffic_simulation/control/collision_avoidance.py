#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class CollisionAvoidance(Node):
    def __init__(self):
        super().__init__("collision_avoidance")

        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10
        )

        self.get_logger().info("Çarpışma Önleme düğümü başlatıldı.")

    def laser_callback(self, scan_data):

        ranges = scan_data.ranges

        safe_distance = 0.5

        front = min(ranges[:30] + ranges[-30:])  
        left = min(ranges[30:90])               
        right = min(ranges[-90:-30])           

        self.get_logger().debug(f"Mesafeler - Ön: {front}, Sol: {left}, Sağ: {right}")

        velocity_msg = Twist()

        if front < safe_distance:
            self.get_logger().warn("Önde engel algılandı! Duruluyor.")
            velocity_msg.linear.x = 0.0
            velocity_msg.angular.z = 0.0

            if left > right:
                self.get_logger().info("Engelden kaçmak için sola dönülüyor.")
                velocity_msg.angular.z = 0.5
            else: 
                self.get_logger().info("Engelden kaçmak için sağa dönülüyor.")
                velocity_msg.angular.z = -0.5
        else:
            self.get_logger().info("Yol açık. İleri hareket ediliyor.")
            velocity_msg.linear.x = 0.5
            velocity_msg.angular.z = 0.0

        self.velocity_publisher.publish(velocity_msg)

        self.get_logger().debug(f"Yayınlanan hız - Linear: {velocity_msg.linear.x}, Angular: {velocity_msg.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    node = CollisionAvoidance()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
