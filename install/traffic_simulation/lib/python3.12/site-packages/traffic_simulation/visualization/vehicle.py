import pygame
import math
import random
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Dict, Optional
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import os

# Explicitly disable car image loading and use polygon drawing
CAR_IMAGE = None
print("Using geometric drawing for vehicles (like in purple_car.py)")

@dataclass
class SensorReading:
    distance: float
    angle: float
    type: str  # 'lidar' or 'radar'
    timestamp: float

@dataclass
class VehicleState:
    position: Tuple[float, float]
    velocity: float
    heading: float
    target_index: int

class Sensor:
    def __init__(self, sensor_type: str, fov: float, max_range: float, num_rays: int):
        self.type = sensor_type
        self.fov = fov  # Field of view in radians
        self.max_range = max_range
        self.num_rays = num_rays
        self.readings: List[SensorReading] = []

    def update(self, vehicle_pos: Tuple[float, float], vehicle_heading: float, 
               obstacles: List[Tuple[float, float]]) -> List[SensorReading]:
        self.readings = []
        angle_step = self.fov / (self.num_rays - 1)
        start_angle = vehicle_heading - self.fov / 2

        for i in range(self.num_rays):
            current_angle = start_angle + i * angle_step
            ray_direction = (math.cos(current_angle), math.sin(current_angle))
            
            # Find closest obstacle in this direction
            min_distance = self.max_range
            for obstacle in obstacles:
                dx = obstacle[0] - vehicle_pos[0]
                dy = obstacle[1] - vehicle_pos[1]
                distance = math.sqrt(dx*dx + dy*dy)
                
                if distance < min_distance:
                    # Check if obstacle is within the ray's angle
                    obstacle_angle = math.atan2(dy, dx)
                    angle_diff = abs(math.atan2(math.sin(obstacle_angle - current_angle),
                                             math.cos(obstacle_angle - current_angle)))
                    if angle_diff < math.pi / 2:  # Within 90 degrees of ray direction
                        min_distance = distance

            self.readings.append(SensorReading(
                distance=min_distance,
                angle=current_angle,
                type=self.type,
                timestamp=pygame.time.get_ticks() / 1000.0
            ))

        return self.readings

class IoTCommunication:
    def __init__(self, vehicle_id: int):
        self.vehicle_id = vehicle_id
        self.communication_range = 100.0  # meters
        self.neighbors: Dict[int, VehicleState] = {}
        self.message_queue: List[Dict] = []

    def broadcast_state(self, state: VehicleState) -> Dict:
        return {
            'vehicle_id': self.vehicle_id,
            'position': state.position,
            'velocity': state.velocity,
            'heading': state.heading,
            'target_index': state.target_index,
            'timestamp': pygame.time.get_ticks() / 1000.0
        }

    def receive_state(self, message: Dict, current_pos: Tuple[float, float]):
        sender_id = message['vehicle_id']
        sender_pos = message['position']
        
        # Check if sender is within range
        dx = sender_pos[0] - current_pos[0]
        dy = sender_pos[1] - current_pos[1]
        distance = math.sqrt(dx*dx + dy*dy)

        if distance <= self.communication_range:
            self.neighbors[sender_id] = VehicleState(
                position=sender_pos,
                velocity=message['velocity'],
                heading=message['heading'],
                target_index=message['target_index']
            )

class SwarmIntelligence:
    def __init__(self):
        # Adjust weights for better coordination
        self.separation_weight = 1.2  # Increased to maintain safer distances
        self.alignment_weight = 0.8   # Increased for better velocity matching
        self.cohesion_weight = 0.4    # Increased for better group behavior
        self.lane_weight = 1.5        # Increased to enforce lane discipline
        self.obstacle_weight = 1.0    # New weight for obstacle avoidance
        
        # Parameters for behavior tuning
        self.min_separation = 10.0    # Minimum safe distance between vehicles
        self.max_separation = 50.0    # Maximum distance to maintain cohesion
        self.velocity_matching_threshold = 5.0  # Speed difference threshold for alignment
        self.lane_change_threshold = 15.0  # Distance threshold for lane changes

    def calculate_behavior(self, vehicle_state: VehicleState, 
                         neighbors: Dict[int, VehicleState],
                         current_lane: int) -> float:
        # Separation: Avoid crowding neighbors with improved distance calculation
        separation_force = self._calculate_separation(vehicle_state, neighbors)
        
        # Alignment: Enhanced velocity matching with relative speed consideration
        alignment_force = self._calculate_alignment(vehicle_state, neighbors)
        
        # Cohesion: Improved group behavior with distance-based weighting
        cohesion_force = self._calculate_cohesion(vehicle_state, neighbors)
        
        # Lane following: Enhanced lane discipline with dynamic lane changes
        lane_force = self._calculate_lane_force(vehicle_state, neighbors, current_lane)
        
        # Combine forces with dynamic weighting based on situation
        total_force = (
            separation_force * self.separation_weight +
            alignment_force * self.alignment_weight +
            cohesion_force * self.cohesion_weight +
            lane_force * self.lane_weight
        )
        
        return total_force

    def _calculate_separation(self, vehicle_state: VehicleState, 
                            neighbors: Dict[int, VehicleState]) -> float:
        if not neighbors:
            return 0.0
            
        separation_force = 0.0
        for neighbor in neighbors.values():
            dx = vehicle_state.position[0] - neighbor.position[0]
            dy = vehicle_state.position[1] - neighbor.position[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance > 0:
                # Enhanced separation force calculation
                if distance < self.min_separation:
                    # Strong repulsion when too close
                    separation_force += (self.min_separation - distance) / distance
                elif distance > self.max_separation:
                    # Slight attraction when too far
                    separation_force -= (distance - self.max_separation) / distance
                    
        return separation_force

    def _calculate_alignment(self, vehicle_state: VehicleState,
                           neighbors: Dict[int, VehicleState]) -> float:
        if not neighbors:
            return 0.0
            
        # Calculate weighted average velocity based on distance
        total_weight = 0.0
        weighted_velocity = 0.0
        
        for neighbor in neighbors.values():
            dx = vehicle_state.position[0] - neighbor.position[0]
            dy = vehicle_state.position[1] - neighbor.position[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance > 0:
                # Weight decreases with distance
                weight = 1.0 / (distance * distance)
                total_weight += weight
                weighted_velocity += neighbor.velocity * weight
                
        if total_weight > 0:
            avg_velocity = weighted_velocity / total_weight
            # Consider relative speed for smoother transitions
            velocity_diff = avg_velocity - vehicle_state.velocity
            if abs(velocity_diff) > self.velocity_matching_threshold:
                return velocity_diff * 0.1
                
        return 0.0

    def _calculate_cohesion(self, vehicle_state: VehicleState,
                          neighbors: Dict[int, VehicleState]) -> float:
        if not neighbors:
            return 0.0
            
        # Calculate weighted center of mass
        total_weight = 0.0
        weighted_x = 0.0
        weighted_y = 0.0
        
        for neighbor in neighbors.values():
            dx = vehicle_state.position[0] - neighbor.position[0]
            dy = vehicle_state.position[1] - neighbor.position[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance > 0:
                # Weight decreases with distance
                weight = 1.0 / (distance * distance)
                total_weight += weight
                weighted_x += neighbor.position[0] * weight
                weighted_y += neighbor.position[1] * weight
                
        if total_weight > 0:
            center_x = weighted_x / total_weight
            center_y = weighted_y / total_weight
            
            # Calculate direction to center
            dx = center_x - vehicle_state.position[0]
            dy = center_y - vehicle_state.position[1]
            return math.atan2(dy, dx)
            
        return 0.0

    def _calculate_lane_force(self, vehicle_state: VehicleState,
                            neighbors: Dict[int, VehicleState],
                            current_lane: int) -> float:
        # Check for vehicles in adjacent lanes
        left_lane_vehicles = []
        right_lane_vehicles = []
        
        for neighbor in neighbors.values():
            if neighbor.target_index < vehicle_state.target_index:
                left_lane_vehicles.append(neighbor)
            elif neighbor.target_index > vehicle_state.target_index:
                right_lane_vehicles.append(neighbor)
        
        # Calculate lane change forces
        lane_force = 0.0
        
        # Stay in right lane by default
        if current_lane == 1:  # Right lane
            lane_force = 0.1  # Slight force to stay right
        elif current_lane == 0:  # Center lane
            # Consider moving to right lane if clear
            if not right_lane_vehicles:
                lane_force = 0.2  # Stronger force to move right
            else:
                lane_force = 0.0  # Stay in center
        else:  # Left lane
            # Consider moving to center if clear
            if not right_lane_vehicles:
                lane_force = -0.2  # Force to move right
            else:
                lane_force = -0.1  # Slight force to move right
        
        # Check for lane change opportunities based on traffic
        if len(neighbors) > 0:
            # Calculate average speed in current lane
            current_lane_speed = sum(n.velocity for n in neighbors.values() 
                                   if n.target_index == current_lane) / len(neighbors)
            
            # If significantly slower than neighbors, consider lane change
            if vehicle_state.velocity < current_lane_speed - 5.0:
                if current_lane > -1 and not left_lane_vehicles:  # Can move left
                    lane_force = -0.3
                elif current_lane < 1 and not right_lane_vehicles:  # Can move right
                    lane_force = 0.3
        
        return lane_force

class Vehicle(Node):
    def __init__(self, position: Tuple[float, float], vehicle_id: int):
        super().__init__(f'vehicle_{vehicle_id}')
        self.id = vehicle_id
        
        # Simple state storage
        self.state = VehicleState(
            position=position,
            velocity=5.0,  # Reduced velocity for more realistic movement
            heading=0.0,
            target_index=0
        )
        
        # Path following
        self.route = []
        self.is_active = True
        self.has_reached_destination = False
        
        # Visual properties - use sizes from purple_car.py
        self.color = (255, 255, 255)  # Default white color
        self.car_length = 60  # Pixels - from purple_car.py
        self.car_width = 30   # Pixels - from purple_car.py
        
        # ROS2 subscribers
        self.create_subscription(Path, f'/vehicle_{vehicle_id}/global_path', 
                               self.path_callback, 10)

    @property
    def position(self):
        return self.state.position
        
    def set_route(self, route: List[Tuple[float, float]]):
        """Set the route for the vehicle"""
        if not route:
            return
            
        self.route = route
        self.state.target_index = 0
        self.is_active = True
        self.has_reached_destination = False
        
        # Set initial heading
        if len(route) > 1:
            dx = route[1][0] - route[0][0]
            dy = route[1][1] - route[0][1]
            self.state.heading = math.atan2(dy, dx)
    
    def update(self, dt: float):
        """Update the vehicle's position and heading"""
        if not self.is_active or not self.route:
            return None
            
        # Get current target
        if self.state.target_index >= len(self.route):
            self.has_reached_destination = True
            self.is_active = False
            return None
            
        target = self.route[self.state.target_index]
        
        # Calculate distance to target
        dx = target[0] - self.state.position[0]
        dy = target[1] - self.state.position[1]
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Check if reached target
        if distance < 15.0:  # Smaller threshold for more precise movement
            self.state.target_index += 1
            if self.state.target_index >= len(self.route):
                self.has_reached_destination = True
                self.is_active = False
                return None
            
            # Update target
            target = self.route[self.state.target_index]
            dx = target[0] - self.state.position[0]
            dy = target[1] - self.state.position[1]
        
        # Update heading - direct to target
        target_heading = math.atan2(dy, dx)
        
        # Smooth heading change (improved from purple_car.py)
        heading_diff = target_heading - self.state.heading
        # Normalize to [-pi, pi]
        heading_diff = math.atan2(math.sin(heading_diff), math.cos(heading_diff))
        
        # Adjust turning speed based on vehicle speed (slower at higher speeds)
        turn_factor = min(0.1, 3.0 / max(1.0, self.state.velocity))
        self.state.heading += heading_diff * turn_factor
        
        # Calculate speed based on turning angle
        # Slow down when turning sharply, go faster on straight paths
        speed_factor = 1.0 - min(0.7, abs(heading_diff) / math.pi)
        current_speed = self.state.velocity * speed_factor
        
        # Move towards target
        self.state.position = (
            self.state.position[0] + math.cos(self.state.heading) * current_speed * dt,
            self.state.position[1] + math.sin(self.state.heading) * current_speed * dt
        )
        
        # Return vehicle state for other vehicles
        return {
            'vehicle_id': self.id,
            'position': self.state.position,
            'velocity': self.state.velocity,
            'heading': self.state.heading,
            'target_index': self.state.target_index,
            'timestamp': pygame.time.get_ticks() / 1000.0
        }
    
    def path_callback(self, msg: Path):
        """Callback for receiving path updates"""
        if not msg.poses:
            self.get_logger().warn("Received empty path")
            return

        self.route = [(pose.pose.position.x, pose.pose.position.y) 
                     for pose in msg.poses]
        self.has_reached_destination = False
        self.is_active = True
        self.state.target_index = 0
        self.get_logger().info(f"Received new path with {len(self.route)} waypoints")

    def draw(self, screen):
        """Draw the vehicle on the screen using the approach from purple_car.py"""
        if not self.is_active:
            return
            
        x, y = self.state.position
        
        # Convert angle to radians for sin/cos calculations
        rad_angle = self.state.heading
        
        # Calculate car center 
        center_x, center_y = x, y
        
        # Calculate corners (unrotated)
        half_width = self.car_length // 2
        half_height = self.car_width // 2
        
        corners = [
            (-half_width, -half_height),  # Top left
            (half_width, -half_height),   # Top right
            (half_width, half_height),    # Bottom right
            (-half_width, half_height)    # Bottom left
        ]
        
        # Rotate and translate corners
        rotated_corners = []
        for corner_x, corner_y in corners:
            # Rotate
            rotated_x = corner_x * math.cos(rad_angle) - corner_y * math.sin(rad_angle)
            rotated_y = corner_x * math.sin(rad_angle) + corner_y * math.cos(rad_angle)
            # Translate
            rotated_corners.append((center_x + rotated_x, center_y + rotated_y))
        
        # Draw car body with black outline
        pygame.draw.polygon(screen, (0, 0, 0), rotated_corners, 0)  # Black outline
        
        # Calculate a slightly smaller polygon for the colored body
        smaller_corners = []
        for corner_x, corner_y in corners:
            # Make it 90% of the original size
            corner_x *= 0.9
            corner_y *= 0.9
            # Rotate
            rotated_x = corner_x * math.cos(rad_angle) - corner_y * math.sin(rad_angle)
            rotated_y = corner_x * math.sin(rad_angle) + corner_y * math.cos(rad_angle)
            # Translate
            smaller_corners.append((center_x + rotated_x, center_y + rotated_y))
        
        # Draw colored body
        pygame.draw.polygon(screen, self.color, smaller_corners, 0)
        
        # Draw front
        front_x = center_x + half_width * math.cos(rad_angle)
        front_y = center_y + half_width * math.sin(rad_angle)
        pygame.draw.circle(screen, (255, 0, 0), (int(front_x), int(front_y)), 8)
        
        # Draw route if available
        if self.route and len(self.route) > 1:
            for i in range(self.state.target_index, len(self.route)-1):
                pygame.draw.line(screen, (0, 255, 0), 
                               self.route[i], 
                               self.route[i+1], 
                               2)  # Thin green line

class VehicleManager:
    def __init__(self, screen_width: int, screen_height: int):
        self.vehicles: Dict[int, Vehicle] = {}
        self.screen_width = screen_width
        self.screen_height = screen_height
        self.next_id = 0
        self.main_vehicle_id = None  # ID of the "main" (purple) vehicle

    def create_vehicle(self, position: Tuple[float, float]) -> Vehicle:
        """Create a new vehicle at the specified position"""
        vehicle = Vehicle(position, self.next_id)
        self.vehicles[self.next_id] = vehicle
        
        # If this is the first vehicle, make it the main (purple) vehicle
        if self.main_vehicle_id is None:
            self.main_vehicle_id = self.next_id
            vehicle.color = (255, 0, 255)  # Purple
        
        self.next_id += 1
        return vehicle
        
    def set_main_vehicle(self, vehicle_id: int):
        """Set a specific vehicle as the main (purple) vehicle"""
        # Reset all vehicles to white
        for v in self.vehicles.values():
            v.color = (255, 255, 255)  # White
            
        # Set the specified vehicle to purple
        if vehicle_id in self.vehicles:
            self.vehicles[vehicle_id].color = (255, 0, 255)  # Purple
            self.main_vehicle_id = vehicle_id

    def update_all(self, dt: float):
        """Update all active vehicles"""
        # Get all vehicle positions
        vehicle_states = {}
        
        # Update each vehicle
        for vehicle in list(self.vehicles.values()):
            if vehicle.is_active:
                state = vehicle.update(dt)
                if state:
                    vehicle_states[vehicle.id] = state
            
    def draw_all(self, screen):
        """Draw all active vehicles"""
        for vehicle in self.vehicles.values():
            if vehicle.is_active:
                vehicle.draw(screen)
            
    def remove_inactive(self):
        """Remove inactive vehicles"""
        self.vehicles = {
            id: v for id, v in self.vehicles.items()
            if (v.is_active and
                0 <= v.state.position[0] <= self.screen_width and
                0 <= v.state.position[1] <= self.screen_height)
        } 