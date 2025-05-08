#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocitySubscriber(Node):
    def __init__(self):
        super().__init__('velocity_subscriber')

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.velocity_callback,
            10
        )

        self.get_logger().info("VelocitySubscriber düğümü başlatıldı.")

    def velocity_callback(self, msg):

        self.get_logger().info(f"Gelen hız komutu - Linear: {msg.linear.x}, Angular: {msg.angular.z}")

def main(args=None):
    rclpy.init(args=args)

    velocity_subscriber_node = VelocitySubscriber()

    try:
        
        rclpy.spin(velocity_subscriber_node)
    except KeyboardInterrupt:
        velocity_subscriber_node.get_logger().info("VelocitySubscriber düğümü kapatılıyor.")
    finally:
        velocity_subscriber_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
