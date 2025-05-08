#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math

class CollisionAvoidance(Node):
    def __init__(self):
        super().__init__('collision_avoidance')
        
        # Subscribe to vehicle positions
        self.vehicle_sub = self.create_subscription(
            Float32MultiArray, 
            '/vehicle_positions', 
            self.vehicle_positions_callback,
            10)
            
        # Publish avoidance commands
        self.avoidance_pub = self.create_publisher(
            Float32MultiArray,
            '/vehicle_commands',
            10)
            
        self.get_logger().info('Collision Avoidance Node started')
        
        # Parameters
        self.safety_distance = 30.0  # Minimum distance between vehicles
        self.look_ahead_distance = 100.0  # Distance to look ahead for potential collisions
        self.vehicle_positions = []
        
    def vehicle_positions_callback(self, msg):
        """Process vehicle position data"""
        # Format of msg.data: [veh1_id, veh1_x, veh1_y, veh1_angle, veh1_speed, veh2_id, ...]
        self.vehicle_positions = []
        
        for i in range(0, len(msg.data), 5):
            if i + 4 < len(msg.data):
                vehicle = {
                    'id': int(msg.data[i]),
                    'x': msg.data[i+1],
                    'y': msg.data[i+2],
                    'angle': msg.data[i+3],
                    'speed': msg.data[i+4]
                }
                self.vehicle_positions.append(vehicle)
                
        self.check_collisions()
        
    def check_collisions(self):
        """Check for potential collisions and publish avoidance commands"""
        if len(self.vehicle_positions) < 2:
            return
            
        commands = []
        
        # Check each vehicle against others
        for i, vehicle in enumerate(self.vehicle_positions):
            slowdown = False
            stop = False
            
            # Calculate forward point (where the vehicle is heading)
            forward_x = vehicle['x'] + math.cos(vehicle['angle']) * self.look_ahead_distance
            forward_y = vehicle['y'] + math.sin(vehicle['angle']) * self.look_ahead_distance
            
            # Check against all other vehicles
            for j, other in enumerate(self.vehicle_positions):
                if i != j:
                    # Calculate distance between vehicles
                    dist = math.sqrt((vehicle['x'] - other['x'])**2 + (vehicle['y'] - other['y'])**2)
                    
                    # If too close, stop
                    if dist < self.safety_distance:
                        stop = True
                        break
                    
                    # Calculate distance to the forward point
                    forward_dist = math.sqrt((forward_x - other['x'])**2 + (forward_y - other['y'])**2)
                    
                    # If potential collision ahead, slow down
                    if forward_dist < self.safety_distance * 2:
                        slowdown = True
            
            # Determine command for this vehicle
            if stop:
                command = 0.0  # Stop
            elif slowdown:
                command = 0.5  # Slow down to half speed
            else:
                command = 1.0  # Full speed
                
            commands.extend([vehicle['id'], command])
        
        # Publish commands
        cmd_msg = Float32MultiArray()
        cmd_msg.data = commands
        self.avoidance_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CollisionAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
