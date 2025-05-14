#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math
import numpy as np

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
            
        self.get_logger().info('Enhanced Collision Avoidance Node started')
        
        # Enhanced parameters
        self.safety_distance = 30.0  # Minimum distance between vehicles
        self.look_ahead_distance = 100.0  # Distance to look ahead for potential collisions
        self.prediction_horizon = 2.0  # Time horizon for collision prediction (seconds)
        self.vehicle_positions = []
        
        # Safety thresholds
        self.emergency_stop_distance = 10.0
        self.slow_down_distance = 20.0
        self.warning_distance = 40.0
        
        # Vehicle dimensions (for more accurate collision detection)
        self.vehicle_length = 4.5  # meters
        self.vehicle_width = 1.8   # meters
        
    def vehicle_positions_callback(self, msg):
        """Process vehicle position data with enhanced state tracking"""
        # Format of msg.data: [veh1_id, veh1_x, veh1_y, veh1_angle, veh1_speed, veh2_id, ...]
        self.vehicle_positions = []
        
        for i in range(0, len(msg.data), 5):
            if i + 4 < len(msg.data):
                vehicle = {
                    'id': int(msg.data[i]),
                    'x': msg.data[i+1],
                    'y': msg.data[i+2],
                    'angle': msg.data[i+3],
                    'speed': msg.data[i+4],
                    'velocity_x': msg.data[i+4] * math.cos(msg.data[i+3]),
                    'velocity_y': msg.data[i+4] * math.sin(msg.data[i+3])
                }
                self.vehicle_positions.append(vehicle)
                
        self.check_collisions()
        
    def predict_position(self, vehicle, time_delta):
        """Predict vehicle position after time_delta seconds"""
        return {
            'x': vehicle['x'] + vehicle['velocity_x'] * time_delta,
            'y': vehicle['y'] + vehicle['velocity_y'] * time_delta
        }
        
    def calculate_time_to_collision(self, vehicle1, vehicle2):
        """Calculate time to collision between two vehicles"""
        # Calculate relative position and velocity
        dx = vehicle2['x'] - vehicle1['x']
        dy = vehicle2['y'] - vehicle1['y']
        dvx = vehicle2['velocity_x'] - vehicle1['velocity_x']
        dvy = vehicle2['velocity_y'] - vehicle1['velocity_y']
        
        # Calculate coefficients for quadratic equation
        a = dvx * dvx + dvy * dvy
        b = 2 * (dx * dvx + dy * dvy)
        c = dx * dx + dy * dy - self.safety_distance * self.safety_distance
        
        if a == 0:  # Vehicles moving at same velocity
            return float('inf')
            
        # Solve quadratic equation
        discriminant = b * b - 4 * a * c
        if discriminant < 0:
            return float('inf')
            
        t1 = (-b + math.sqrt(discriminant)) / (2 * a)
        t2 = (-b - math.sqrt(discriminant)) / (2 * a)
        
        # Return smallest positive time
        if t1 > 0 and t2 > 0:
            return min(t1, t2)
        elif t1 > 0:
            return t1
        elif t2 > 0:
            return t2
        return float('inf')
        
    def check_collisions(self):
        """Enhanced collision detection with prediction and safety measures"""
        if len(self.vehicle_positions) < 2:
            return
            
        commands = []
        
        # Check each vehicle against others
        for i, vehicle in enumerate(self.vehicle_positions):
            emergency_stop = False
            slowdown = False
            warning = False
            
            # Calculate forward point (where the vehicle is heading)
            forward_x = vehicle['x'] + math.cos(vehicle['angle']) * self.look_ahead_distance
            forward_y = vehicle['y'] + math.sin(vehicle['angle']) * self.look_ahead_distance
            
            # Check against all other vehicles
            for j, other in enumerate(self.vehicle_positions):
                if i != j:
                    # Calculate current distance
                    dist = math.sqrt((vehicle['x'] - other['x'])**2 + 
                                   (vehicle['y'] - other['y'])**2)
                    
                    # Calculate time to collision
                    ttc = self.calculate_time_to_collision(vehicle, other)
                    
                    # Predict positions
                    pred_vehicle = self.predict_position(vehicle, self.prediction_horizon)
                    pred_other = self.predict_position(other, self.prediction_horizon)
                    pred_dist = math.sqrt((pred_vehicle['x'] - pred_other['x'])**2 + 
                                        (pred_vehicle['y'] - pred_other['y'])**2)
                    
                    # Determine threat level
                    if dist < self.emergency_stop_distance or ttc < 1.0:
                        emergency_stop = True
                        break
                    elif dist < self.slow_down_distance or ttc < 2.0:
                        slowdown = True
                    elif dist < self.warning_distance or ttc < 3.0:
                        warning = True
                    
                    # Check predicted collision
                    if pred_dist < self.safety_distance:
                        slowdown = True
            
            # Determine command for this vehicle
            if emergency_stop:
                command = 0.0  # Stop
            elif slowdown:
                command = 0.3  # Slow down significantly
            elif warning:
                command = 0.7  # Slight slowdown
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
