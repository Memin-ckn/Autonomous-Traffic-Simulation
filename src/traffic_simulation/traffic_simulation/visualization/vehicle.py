import pygame
import math
import random

class Vehicle:
    def __init__(self, position, vehicle_type="car", color=(0, 150, 255), max_speed=5.0):
        self.id = id(self)  # Unique ID for each vehicle
        self.position = position
        self.prev_position = position
        self.destination = None
        self.type = vehicle_type
        self.color = color
        self.speed = 0.0
        self.max_speed = max_speed
        self.acceleration = 0.1
        self.route = []
        self.current_route_index = 0
        self.size = self._get_size_for_type()
        self.angle = 0
        self.is_active = True
        
        # Collision avoidance
        self.avoid_factor = 1.0  # Speed multiplier from collision avoidance (1.0 = normal, 0.5 = slow, 0.0 = stop)
        self.safety_radius = self.size * 3  # Minimum distance to maintain from other vehicles
        self.detection_radius = self.size * 10  # Distance to detect other vehicles
        
    def _get_size_for_type(self):
        if self.type == "car":
            return 8
        elif self.type == "truck":
            return 12
        elif self.type == "bus":
            return 15
        return 8

    def set_route(self, route):
        self.route = route
        self.current_route_index = 0
        if len(route) > 0:
            self.destination = route[0]

    def update(self):
        if not self.is_active or not self.destination:
            return

        # Calculate direction vector
        dx = self.destination[0] - self.position[0]
        dy = self.destination[1] - self.position[1]
        distance = math.sqrt(dx*dx + dy*dy)

        # Update angle for drawing
        if distance > 0:
            self.angle = math.atan2(dy, dx)

        # If we're close to the destination, move to next point in route
        if distance < self.speed:
            self.position = self.destination
            self.current_route_index += 1
            if self.current_route_index < len(self.route):
                self.destination = self.route[self.current_route_index]
            else:
                self.is_active = False
                return
        else:
            # Normalize direction vector and move
            dx /= distance
            dy /= distance
            
            # Accelerate until max speed, applying avoidance factor
            target_speed = self.max_speed * self.avoid_factor
            
            if self.speed < target_speed:
                self.speed += self.acceleration
            elif self.speed > target_speed:
                self.speed -= self.acceleration * 2  # Decelerate faster than accelerate
                
            # Clamp speed
            self.speed = max(0.0, min(self.speed, self.max_speed))
            
            # Update position
            self.prev_position = self.position
            new_x = self.position[0] + dx * self.speed
            new_y = self.position[1] + dy * self.speed
            self.position = (new_x, new_y)

    def detect_collision(self, other_vehicles):
        """Check for potential collisions with other vehicles and adjust speed"""
        self.avoid_factor = 1.0  # Reset to normal speed
        
        # Get our forward vector (where we're heading)
        if not self.destination:
            return
        
        forward_x = math.cos(self.angle)
        forward_y = math.sin(self.angle)
        
        for vehicle in other_vehicles:
            # Skip ourselves
            if vehicle.id == self.id or not vehicle.is_active:
                continue
                
            # Calculate distance between vehicles
            dx = vehicle.position[0] - self.position[0]
            dy = vehicle.position[1] - self.position[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            # If within detection radius
            if distance < self.detection_radius:
                # Calculate dot product to see if the other vehicle is in front of us
                dot_product = dx * forward_x + dy * forward_y
                
                # Other vehicle is ahead of us in our direction
                if dot_product > 0:
                    # The closer they are, the more we slow down
                    if distance < self.safety_radius:
                        self.avoid_factor = 0.0  # Stop completely
                    elif distance < self.safety_radius * 2:
                        self.avoid_factor = 0.3  # Slow significantly 
                    elif distance < self.safety_radius * 3:
                        self.avoid_factor = 0.7  # Slow moderately
    
    def draw(self, screen):
        if not self.is_active:
            return
            
        # Draw vehicle as a circle for now
        pygame.draw.circle(screen, self.color, (int(self.position[0]), int(self.position[1])), self.size)
        
        # Draw direction indicator
        end_x = self.position[0] + math.cos(self.angle) * self.size
        end_y = self.position[1] + math.sin(self.angle) * self.size
        pygame.draw.line(screen, (0, 0, 0), 
                         (int(self.position[0]), int(self.position[1])), 
                         (int(end_x), int(end_y)), 2)

class VehicleManager:
    def __init__(self, screen_width, screen_height):
        self.vehicles = []
        self.screen_width = screen_width
        self.screen_height = screen_height
        
    def create_vehicle(self, start_pos, vehicle_type="car", color=None):
        if color is None:
            color = (
                random.randint(50, 200),
                random.randint(50, 200),
                random.randint(50, 200)
            )
        vehicle = Vehicle(start_pos, vehicle_type, color)
        self.vehicles.append(vehicle)
        return vehicle
        
    def update_all(self):
        # First, let all vehicles detect potential collisions
        for vehicle in self.vehicles:
            vehicle.detect_collision(self.vehicles)
            
        # Then update all vehicles
        for vehicle in self.vehicles:
            vehicle.update()
            
    def draw_all(self, screen):
        for vehicle in self.vehicles:
            vehicle.draw(screen)
            
    def remove_inactive(self):
        self.vehicles = [v for v in self.vehicles if v.is_active]
        
    def get_vehicle_positions(self):
        """Get all vehicle positions for ROS publishing"""
        positions = []
        for vehicle in self.vehicles:
            if vehicle.is_active:
                positions.extend([
                    vehicle.id,
                    vehicle.position[0],
                    vehicle.position[1],
                    vehicle.angle,
                    vehicle.speed
                ])
        return positions 