#!/usr/bin/env python3

import pygame
import sys
import math
import random
import rclpy
from rclpy.node import Node

# Debug import issues
try:
    print("Importing vehicle classes...")
    from traffic_simulation.vehicles.vehicle import Vehicle, WhiteCar, SmartCar
    print("Importing sensor classes...")
    from traffic_simulation.sensors.sensor import Lidar, Radar
    print("Importing communication classes...")
    from traffic_simulation.communication.iot import IoTCommunicator
    print("Importing collision avoidance classes...")
    from traffic_simulation.planning.collision_avoidance import CollisionAvoidance, CollisionStrategy
    print("All imports successful!")
except Exception as e:
    print(f"Import error: {e}")
    import traceback
    traceback.print_exc()
    # Don't crash, but set flags to avoid using these features
    IMPORT_ERROR = True
else:
    IMPORT_ERROR = False

class SimpleVisualizer(Node):
    def __init__(self):
        super().__init__('simple_visualizer')
        pygame.init()
        
        # Create a fullscreen window
        self.screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
        self.width, self.height = self.screen.get_size()
        pygame.display.set_caption("Traffic Simulator")
        
        # Define colors
        self.GREEN = (34, 139, 34)  # Background
        self.BLACK = (0, 0, 0)      # Road, outlines
        self.WHITE = (255, 255, 255) # Road lines and cars
        self.RED = (255, 0, 0)      # Start point
        self.BLUE = (0, 0, 255)     # End point
        self.PURPLE = (255, 0, 255) # Main car color
        self.YELLOW = (255, 255, 0) # Highlights
        
        # Import CollisionStrategy enum
        try:
            from traffic_simulation.planning.collision_avoidance import CollisionStrategy
            self.CollisionStrategy = CollisionStrategy
        except Exception as e:
            print(f"Failed to import CollisionStrategy: {e}")
            # Define a backup enum
            class CollisionStrategyBackup:
                SLOW_DOWN = 1
                REROUTE = 2
                YIELD = 3
            self.CollisionStrategy = CollisionStrategyBackup
        
        # Grid size dropdown
        self.font = pygame.font.SysFont('Arial', int(self.height / 30))
        self.grid_sizes = [3, 4, 5, 6, 7, 8]
        self.current_grid_size = 4  # Default grid size
        self.dropdown_open = False
        
        # Car count dropdown
        self.car_counts = [1, 2, 3, 4, 5, 6, 7, 8]
        self.current_car_count = 4  # Default car count
        self.car_dropdown_open = False
        
        # Dropdown dimensions
        self.dropdown_width = int(self.width / 6)
        self.dropdown_height = int(self.height / 20)
        
        # Car properties
        self.CAR_WIDTH = 60
        self.CAR_HEIGHT = 30
        
        # Main car - using SmartCar class
        self.smart_car = None
        self.main_car_speed = 3.0
        
        # White cars
        self.white_cars = []
        
        # Route
        self.start_point = None
        self.end_point = None
        self.route = []
        self.current_target = 0
        
        # Road width
        self.ROAD_WIDTH = 40
        
        # Debug options
        self.show_hitboxes = False  # Toggle with 'H' key
        
        # Detection and avoidance parameters
        self.detection_range = self.ROAD_WIDTH * 12  # Increase detection zone (was 10)
        self.safety_zone_size = 2.0  # Increase safety zone multiplier (was 1.5)
        self.detection_zone_size = 4.0  # Increase detection zone size (was 3.5)
        
        # Movement smoothing parameters
        self.acceleration_rate = 0.05  # How quickly cars accelerate
        self.deceleration_rate = 0.1   # Increase deceleration rate for quicker stopping (was 0.08)
        self.min_speed = 0.1  # Minimum speed when slowing down
        
        # Route planning parameters
        self.route_check_interval = 30  # Check route every 30 frames (about once per second)
        self.preventative_reroute_enabled = True  # Enable preventative rerouting
        self.route_safety_threshold = 50  # Minimum safety score to avoid rerouting
        self.route_check_count = 0  # Counter for checking route
        
        # Generate the map
        self.intersections, self.roads = self.generate_grid()
        
        # Create white cars
        self.white_cars = self.create_white_cars()  # Create 8 white cars
        
        # For FPS control
        self.clock = pygame.time.Clock()
    
    def generate_grid(self):
        """
        Generate a grid of intersections and roads
        Returns:
            - intersections: list of (x,y) tuple points
            - roads: list of ((x1,y1), (x2,y2)) tuple of tuples representing road segments
        """
        grid_size = self.current_grid_size  # Use the current grid size
        intersections = []
        roads = []
        
        # Create a grid of intersection points
        cell_width = self.width // (grid_size + 1)
        cell_height = self.height // (grid_size + 1)
        
        for i in range(grid_size):
            for j in range(grid_size):
                x = (j + 1) * cell_width
                y = (i + 1) * cell_height
                intersections.append((x, y))
        
        # Connect adjacent intersections with roads
        for i in range(len(intersections)):
            for j in range(i+1, len(intersections)):
                x1, y1 = intersections[i]
                x2, y2 = intersections[j]
                
                # Only connect if they're adjacent (within certain distance)
                dx = abs(x1 - x2)
                dy = abs(y1 - y2)
                
                # Add some randomness to road placement
                if (dx <= cell_width * 1.5 and dy <= cell_height * 1.5) and random.random() < 0.7:
                    # Store as tuple of tuples to ensure proper lookup
                    road = (tuple(intersections[i]), tuple(intersections[j]))
                    roads.append(road)
        
        # Debug output
        print(f"Generated grid with {len(intersections)} intersections and {len(roads)} roads")
        return intersections, roads
    
    def find_path(self, start, end):
        """
        Find a path from start to end using only existing roads
        Uses breadth-first search
        """
        if start == end:
            return [start]
            
        # Check if there's a direct road between start and end
        if self.has_road_between(start, end):
            return [start, end]
        
        # Build an adjacency list based on actual roads
        adjacency = {}
        for point in self.intersections:
            adjacency[point] = []
            
        # Add connected points based on actual roads
        for road in self.roads:
            a, b = road
            # Add both directions
            if a in adjacency:
                adjacency[a].append(b)
            if b in adjacency:
                adjacency[b].append(a)
        
        # Check if either start or end is not in adjacency list
        if start not in adjacency or end not in adjacency:
            return None  # No path possible
        
        # Breadth-first search
        visited = set()
        queue = [(start, [start])]
        
        while queue:
            current, path = queue.pop(0)
            
            if current == end:
                # Verify each segment is a valid road
                for i in range(len(path)-1):
                    if not self.has_road_between(path[i], path[i+1]):
                        print(f"Invalid segment: {path[i]} to {path[i+1]}")
                        return None  # Invalid path segment
                return path
                
            if current in visited:
                continue
                
            visited.add(current)
            
            # Add neighbors from adjacency list to ensure only existing roads are used
            for neighbor in adjacency.get(current, []):
                if neighbor not in visited:
                    queue.append((neighbor, path + [neighbor]))
        
        return None  # No path found
    
    def create_white_cars(self, count=None):
        """Create white cars with valid routes"""
        # Use the current_car_count if count is not specified
        if count is None:
            count = self.current_car_count
            
        cars = []
        for _ in range(count):
            route = self.generate_random_route()
            
            # Validate route one more time
            valid_route = True
            for i in range(len(route) - 1):
                if not self.has_road_between(route[i], route[i+1]):
                    print(f"Invalid route generated: {route[i]} to {route[i+1]}")
                    valid_route = False
                    break
            
            # If invalid, try once more with a simple route
            if not valid_route and self.roads:
                road = self.roads[0]  # Take the first road
                route = list(road)    # Convert tuple to list
            
            # Use our WhiteCar class from the vehicles module
            car = WhiteCar(route[0][0], route[0][1], route, self)
            
            # Pass map data to the car
            from traffic_simulation.vehicles.vehicle import WhiteCar as RealWhiteCar
            if isinstance(car.vehicle, RealWhiteCar):
                car.vehicle.set_map_data(self.intersections, self.roads)
            
            cars.append(car)
        return cars
    
    def generate_random_route(self):
        """Generate a valid route along roads for vehicles to follow"""
        # Check if we have roads
        if not self.roads or not self.intersections:
            print("No roads or intersections available")
            # Return a default position
            if self.intersections:
                return [self.intersections[0]]
            return [(self.width // 2, self.height // 2)]
            
        # Try to find valid start/end points with a path
        attempts = 0
        max_attempts = min(30, len(self.intersections) * 2)  # Scale with grid size
        
        while attempts < max_attempts:
            attempts += 1
            
            # Select random start and end intersections
            start = random.choice(self.intersections)
            end = random.choice(self.intersections)
            
            # Skip if same point
            if start == end:
                continue
                
            # Find path
            path = self.find_path(start, end)
            
            # Check if path is valid and has at least two points
            if path and len(path) >= 2:
                # Double-check that consecutive points have roads between them
                valid_path = True
                for i in range(len(path) - 1):
                    if not self.has_road_between(path[i], path[i+1]):
                        valid_path = False
                        break
                
                if valid_path:
                    return path
        
        # If we still don't have a valid path, find a single valid road segment
        for road in self.roads:
            return list(road)  # Return a list of the two endpoints
            
        # Last resort fallback - just return a single point
        # This will make the car effectively stay in place
        return [self.intersections[0]]
    
    def draw_car(self, x, y, angle, color):
        # Convert angle from degrees to radians
        rad_angle = math.radians(angle)
        
        # Calculate car center
        center_x, center_y = x, y
        
        # Calculate corners (unrotated)
        half_width = self.CAR_WIDTH // 2
        half_height = self.CAR_HEIGHT // 2
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
        pygame.draw.polygon(self.screen, self.BLACK, rotated_corners, 0)  # Black outline
        
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
        pygame.draw.polygon(self.screen, color, smaller_corners, 0)
        
        # Draw front
        front_x = center_x + half_width * math.cos(rad_angle)
        front_y = center_y + half_width * math.sin(rad_angle)
        pygame.draw.circle(self.screen, self.RED, (int(front_x), int(front_y)), 8)
    
    def update_main_car(self):
        if self.smart_car is None or self.current_target >= len(self.route) or not self.route:
            return
        
        try:
            # If the route changed, update the smart car's route
            if self.smart_car.route != self.route:
                self.smart_car.route = self.route.copy()
                self.smart_car.current_target = self.current_target
                self.smart_car.original_route = self.route.copy()  # Save original route for rerouting
            
            # Reset collision risk flag
            self.smart_car.collision_risk = False
            
            # Get real vehicle objects for sensor detection
            vehicle_list = [car.vehicle for car in self.white_cars]
            
            # Periodically check if we need preventative rerouting (like a flu shot)
            self.route_check_count += 1
            if self.preventative_reroute_enabled and self.route_check_count >= self.route_check_interval:
                self.route_check_count = 0
                is_rerouted = self.preventative_reroute()
                if is_rerouted:
                    # If we've already rerouted preventatively, skip the rest to avoid double-handling
                    # Update our target tracking to match the smart car
                    self.current_target = self.smart_car.current_target
                    return
            
            # Pre-check for potential threats - white cars in vicinity even without predicted collisions
            for car in self.white_cars:
                dx = car.x - self.smart_car.x
                dy = car.y - self.smart_car.y
                distance = math.sqrt(dx*dx + dy*dy)
                
                # Just being in the vicinity will cause some caution
                if distance <= self.ROAD_WIDTH * 6:  # Increased caution zone 
                    self.smart_car.collision_risk = True
                    # Set an initial strategy of slowing down just due to proximity
                    if not hasattr(self.smart_car, 'current_strategy') or self.smart_car.current_strategy is None:
                        self.smart_car.current_strategy = CollisionStrategy.SLOW_DOWN
            
            # Reactively check for imminent collisions (treatment, not prevention)
            # This handles cases where preventative measures failed or weren't enough
            collisions, predicted_collisions = self.check_all_collisions()
            
            if collisions:
                print(f"EMERGENCY: Immediate collision detected with {len(collisions)} cars!")
            
            if predicted_collisions:
                closest_collision = min(predicted_collisions, key=lambda x: x[2])
                _, _, time = closest_collision
                print(f"WARNING: Collision predicted in {time} steps - immediate action needed")
            
            # Store current target speed for smooth adjustment
            target_speed = self.smart_car.default_speed

            # Strict pre-move collision check: simulate next position and block move if collision would occur
            # Only do this if speed > 0 (i.e., car is trying to move)
            if self.smart_car.speed > 0:
                # Calculate next position based on current speed and angle
                dt = 1  # Assume 1 time step for now (can be adjusted if needed)
                rad_angle = math.radians(self.smart_car.angle)
                next_x = self.smart_car.x + math.cos(rad_angle) * self.smart_car.speed * dt
                next_y = self.smart_car.y + math.sin(rad_angle) * self.smart_car.speed * dt
                # Create a temporary car object at the next position
                class TempCar:
                    def __init__(self, x, y, angle, car_id=None):
                        self.x = x
                        self.y = y
                        self.angle = angle
                        self.id = car_id
                temp_car = TempCar(next_x, next_y, self.smart_car.angle, car_id="purple_car")
                # Check for collision with any white car, but only yield if the white car is approaching from the side
                overlap = False
                for car in self.white_cars:
                    # Calculate angle difference
                    if hasattr(car, 'angle'):
                        angle_diff = abs((self.smart_car.angle - car.angle + 180) % 360 - 180)
                        # Only yield if the white car is approaching from the side (angle_diff < 75) or from behind (angle_diff > 105 and < 255)
                        # Ignore if the white car is in front (75 <= angle_diff <= 105)
                        if angle_diff < 75 or (angle_diff > 105 and angle_diff < 255):
                            if self.check_collision(temp_car, car):
                                overlap = True
                                break
                if overlap:
                    # Block movement and set speed to zero
                    self.smart_car.speed = 0.0
                    self.smart_car.collision_risk = True
                    self.smart_car.current_strategy = CollisionStrategy.YIELD
                    # Do not update position, skip the rest of the update
                    return

            # Check if we're approaching a turn and need to slow down
            turn_slowdown_factor = 1.0  # Default: no slowdown
            approaching_turn = False
            completed_turn = False  # New flag for turn completion
            
            # Get current position and direction
            current_pos = (self.smart_car.x, self.smart_car.y)
            current_angle = self.smart_car.angle
            current_dir_vector = (math.cos(math.radians(current_angle)), math.sin(math.radians(current_angle)))
            
            # Check if there's an upcoming turn
            if self.current_target < len(self.route) - 1:
                # Current target and next target to determine the upcoming turn
                current_target_pt = self.route[self.current_target]
                next_target_pt = self.route[self.current_target + 1]
                
                # Distance to current target (waypoint)
                dx_to_waypoint = current_target_pt[0] - current_pos[0]
                dy_to_waypoint = current_target_pt[1] - current_pos[1]
                dist_to_waypoint = math.sqrt(dx_to_waypoint*dx_to_waypoint + dy_to_waypoint*dy_to_waypoint)
                
                # Increase turn awareness distance for earlier slowdown
                turn_awareness_distance = self.ROAD_WIDTH * 6  # Increased from 4 to 6
                
                if dist_to_waypoint < turn_awareness_distance:
                    # Calculate direction vectors for current segment and next segment
                    current_segment_dx = current_target_pt[0] - current_pos[0]
                    current_segment_dy = current_target_pt[1] - current_pos[1]
                    
                    next_segment_dx = next_target_pt[0] - current_target_pt[0]
                    next_segment_dy = next_target_pt[1] - current_target_pt[1]
                    
                    # Normalize the vectors (make them unit vectors)
                    current_segment_len = math.sqrt(current_segment_dx*current_segment_dx + current_segment_dy*current_segment_dy)
                    next_segment_len = math.sqrt(next_segment_dx*next_segment_dx + next_segment_dy*next_segment_dy)
                    
                    if current_segment_len > 0 and next_segment_len > 0:
                        current_segment_dx /= current_segment_len
                        current_segment_dy /= current_segment_len
                        
                        next_segment_dx /= next_segment_len
                        next_segment_dy /= next_segment_len
                        
                        # Calculate the dot product to find the angle between segments
                        dot_product = current_segment_dx*next_segment_dx + current_segment_dy*next_segment_dy
                        dot_product = max(-1.0, min(1.0, dot_product))  # Clamp to [-1, 1]
                        
                        # Calculate the angle between the segments in degrees
                        angle_between_segments = math.degrees(math.acos(dot_product))
                        
                        # If there's a significant turn ahead, slow down based on turn sharpness
                        if angle_between_segments > 20:  # Only consider as turn if angle > 20 degrees
                            approaching_turn = True
                            
                            # Calculate slowdown factor - sharper turns require slower speeds
                            # Linear interpolation: 0 degrees = 1.0 (no slowdown), 90+ degrees = 0.25 (75% slowdown)
                            max_slowdown = 0.25  # Increased slowdown for sharp turns (was 0.3)
                            turn_slowdown_factor = 1.0 - ((angle_between_segments / 90.0) * (1.0 - max_slowdown))
                            turn_slowdown_factor = max(max_slowdown, turn_slowdown_factor)
                            
                            # Adjust slowdown based on distance to turn (more gradual slowdown)
                            # Use quadratic interpolation for smoother deceleration
                            distance_factor = (dist_to_waypoint / turn_awareness_distance)
                            distance_factor = distance_factor * distance_factor  # Square for more gradual initial slowdown
                            
                            # Gradually apply the slowdown as we approach the turn
                            effective_slowdown = 1.0 - (1.0 - turn_slowdown_factor) * (1.0 - distance_factor)
                            turn_slowdown_factor = max(turn_slowdown_factor, effective_slowdown)
                            
                            # Additional slowdown for very sharp turns
                            if angle_between_segments > 60:
                                turn_slowdown_factor *= 0.8  # Extra 20% reduction for sharp turns
                    else:
                        # Check if we've completed a turn and can speed up
                        # Calculate angle to next target
                        if self.current_target < len(self.route):
                            target = self.route[self.current_target]
                            dx_to_target = target[0] - current_pos[0]
                            dy_to_target = target[1] - current_pos[1]
                            target_angle = math.degrees(math.atan2(dy_to_target, dx_to_target))
                            
                            # If we're aligned with the target (within 15 degrees)
                            angle_diff = abs((target_angle - current_angle + 180) % 360 - 180)
                            if angle_diff < 15:
                                completed_turn = True
                                # Allow faster acceleration after completing turn
                                self.acceleration_rate = 0.08  # Temporarily increase acceleration
                            else:
                                self.acceleration_rate = 0.05  # Reset to normal acceleration
            
            # Adjust target speed based on collision status
            if self.smart_car.collision_risk:
                if self.smart_car.current_strategy == CollisionStrategy.YIELD:
                    # Complete stop only for emergency situations
                    target_speed = 0
                elif self.smart_car.current_strategy == CollisionStrategy.SLOW_DOWN:
                    # Calculate a target speed based on time to collision or other factors
                    if predicted_collisions:
                        closest_collision = min(predicted_collisions, key=lambda x: x[2])
                        _, _, time_to_collision = closest_collision
                        # The closer the collision, the slower we go - make slowdown more aggressive
                        slowdown_factor = max(0.05, time_to_collision / 10.0)  # More aggressive slowdown
                        target_speed = self.smart_car.default_speed * slowdown_factor
                    else:
                        # Even with no specific predicted collision, slow down just due to proximity
                        target_speed = self.smart_car.default_speed * 0.6  # 60% of normal speed
            
            # Apply turn slowdown if necessary and it results in a lower speed than other factors
            if approaching_turn:
                turn_target_speed = self.smart_car.default_speed * turn_slowdown_factor
                target_speed = min(target_speed, turn_target_speed)
            elif completed_turn:
                # After completing turn, gradually return to normal speed
                target_speed = max(target_speed, self.smart_car.default_speed * 0.8)  # Allow at least 80% speed
                
                # Add visual debug information if needed
                if hasattr(self, 'show_hitboxes') and self.show_hitboxes:
                    print(f"Completed turn: Speed = {target_speed:.1f}/{self.smart_car.default_speed:.1f}")
            
            # Smoothly adjust the actual speed toward the target speed
            if self.smart_car.speed < target_speed:
                # Accelerate gradually
                self.smart_car.speed = min(self.smart_car.speed + self.acceleration_rate, target_speed)
            elif self.smart_car.speed > target_speed:
                # Decelerate gradually - make deceleration faster for safety
                self.smart_car.speed = max(self.smart_car.speed - self.deceleration_rate * 1.5, target_speed)
                # Ensure we don't go below minimum speed unless target is 0
                if target_speed > 0:
                    self.smart_car.speed = max(self.smart_car.speed, self.min_speed)
            
            # Only update if no immediate collisions or if we're rerouting/yielding
            if not collisions or self.smart_car.current_strategy in [CollisionStrategy.REROUTE, CollisionStrategy.YIELD]:
                # Update the smart car with collision avoidance
                self.smart_car.update(vehicle_list)
                
                # Update our target tracking to match the smart car
                self.current_target = self.smart_car.current_target
                
                # Check if route has been modified by collision avoidance
                if self.smart_car.route != self.route:
                    self.route = self.smart_car.route
            
            # After moving, check if we're still at risk of collision
            if not self.smart_car.collision_risk:
                # Re-check for collisions after moving
                collisions, predicted_collisions = self.check_all_collisions()
                
                # If we still have collision risk, we might need more aggressive avoidance
                if collisions or predicted_collisions:
                    # Try to find a better route if currently using SLOW_DOWN
                    if self.smart_car.current_strategy == CollisionStrategy.SLOW_DOWN and len(predicted_collisions) > 0:
                        # Get closest collision
                        closest_collision = min(predicted_collisions, key=lambda x: x[2])
                        _, obstacle_car, _ = closest_collision
                        
                        # Try to reroute - this is now reactive (like medicine after getting sick)
                        self.initiate_reroute(self.smart_car, obstacle_car)
                
        except Exception as e:
            print(f"Error updating SmartCar: {e}")
            import traceback
            traceback.print_exc()
    
    def draw_scene(self):
        # Clear screen
        self.screen.fill(self.GREEN)
        
        # Draw roads
        for road in self.roads:
            start, end = road
            pygame.draw.line(self.screen, self.BLACK, start, end, self.ROAD_WIDTH)  # Thick black road
            pygame.draw.line(self.screen, self.WHITE, start, end, 2)   # Thin white center line
        
        # Draw intersections
        for intersection in self.intersections:
            pygame.draw.circle(self.screen, self.BLACK, intersection, 15)
            pygame.draw.circle(self.screen, (180, 180, 180), intersection, 12)
        
        # Draw each white car's current target
        for car in self.white_cars:
            if len(car.route) > car.current_target:
                target = car.route[car.current_target]
                pygame.draw.circle(self.screen, (255, 165, 0), target, 6)  # Orange circle for target
        
        # Draw start and end points
        if self.start_point:
            pygame.draw.circle(self.screen, self.RED, self.start_point, 20)
        if self.end_point:
            pygame.draw.circle(self.screen, self.BLUE, self.end_point, 20)
        
        # Draw route for main car
        if self.route:
            for i in range(len(self.route) - 1):
                pygame.draw.line(self.screen, self.PURPLE, self.route[i], self.route[i+1], 2)
            
            # Highlight current target
            if self.current_target < len(self.route):
                pygame.draw.circle(self.screen, self.RED, self.route[self.current_target], 8)
                
        # Draw routes for white cars (for debugging)
        for car in self.white_cars:
            if len(car.route) > 1:
                for i in range(len(car.route) - 1):
                    # Check if this segment is valid
                    if self.has_road_between(car.route[i], car.route[i+1]):
                        # Valid segment - draw thin green line
                        pygame.draw.line(self.screen, (0, 200, 0), car.route[i], car.route[i+1], 1)
                    else:
                        # Invalid segment - draw thick red line
                        pygame.draw.line(self.screen, (255, 0, 0), car.route[i], car.route[i+1], 3)
                        # Add red X at problem points
                        midpoint = ((car.route[i][0] + car.route[i+1][0])/2, 
                                    (car.route[i][1] + car.route[i+1][1])/2)
                        size = 15
                        pygame.draw.line(self.screen, (255, 0, 0), 
                                        (midpoint[0]-size, midpoint[1]-size),
                                        (midpoint[0]+size, midpoint[1]+size), 5)
                        pygame.draw.line(self.screen, (255, 0, 0), 
                                        (midpoint[0]-size, midpoint[1]+size),
                                        (midpoint[0]+size, midpoint[1]-size), 5)
        
        # Draw white cars - now with "ghost" visualization
        for car in self.white_cars:
            # Draw ghost danger zone (only when hitboxes are shown)
            if hasattr(self, 'show_hitboxes') and self.show_hitboxes:
                # Draw a semi-transparent danger zone around ghosts
                danger_radius = self.ROAD_WIDTH * 5
                ghost_surface = pygame.Surface((danger_radius*2, danger_radius*2), pygame.SRCALPHA)
                pygame.draw.circle(ghost_surface, (255, 0, 0, 40), (danger_radius, danger_radius), danger_radius)
                self.screen.blit(ghost_surface, (car.x - danger_radius, car.y - danger_radius))
            
            # Draw the white car
            self.draw_car(car.x, car.y, car.angle, self.WHITE)
            
            # Draw hitbox for debugging
            if hasattr(self, 'show_hitboxes') and self.show_hitboxes:
                corners = self.get_car_corners(car.x, car.y, car.angle)
                pygame.draw.lines(self.screen, (255, 165, 0), True, corners, 2)
                
                # Draw safety zone around white cars (slightly larger hitbox)
                if self.show_hitboxes and self.smart_car is not None:
                    safety_corners = self.get_car_corners(car.x, car.y, car.angle, safety_margin=self.safety_zone_size)
                    pygame.draw.lines(self.screen, (255, 100, 0), True, safety_corners, 1)
        
        # Draw main car (purple) - now with Pacman-like features and turn indicator
        if self.smart_car is not None:
            collision_color = self.PURPLE
            
            # Change car color based on collision status and strategy
            if hasattr(self.smart_car, 'collision_risk') and self.smart_car.collision_risk:
                # Red tint for collision risk - "powered-up Pacman" look
                collision_color = (255, 50, 255)  # Brighter purple
                
                # Draw collision avoidance indicator
                if hasattr(self.smart_car, 'current_strategy'):
                    indicator_x = self.smart_car.x
                    indicator_y = self.smart_car.y - 40  # Above car
                    
                    # Different indicators based on strategy
                    if self.smart_car.current_strategy == CollisionStrategy.SLOW_DOWN:
                        # Draw yellow/orange slow down indicator
                        pygame.draw.circle(self.screen, (255, 165, 0), (int(indicator_x), int(indicator_y)), 15)
                        font = pygame.font.SysFont('Arial', 14)
                        text = font.render("SLOW", True, self.BLACK)
                        self.screen.blit(text, (indicator_x - 15, indicator_y - 7))
                        
                    elif self.smart_car.current_strategy == CollisionStrategy.YIELD:
                        # Draw red stop indicator
                        pygame.draw.circle(self.screen, (255, 0, 0), (int(indicator_x), int(indicator_y)), 15)
                        font = pygame.font.SysFont('Arial', 14)
                        text = font.render("STOP", True, self.WHITE)
                        self.screen.blit(text, (indicator_x - 15, indicator_y - 7))
                        
                    elif self.smart_car.current_strategy == CollisionStrategy.REROUTE:
                        # Draw blue reroute indicator
                        pygame.draw.circle(self.screen, (0, 100, 255), (int(indicator_x), int(indicator_y)), 15)
                        font = pygame.font.SysFont('Arial', 14)
                        text = font.render("REROUTE", True, self.WHITE)
                        self.screen.blit(text, (indicator_x - 25, indicator_y - 7))
            
            # Check if we're slowing down for a turn
            if self.current_target < len(self.route) - 1 and self.smart_car.speed < self.smart_car.default_speed * 0.9:
                # Display turn indicator only if we're significantly slowing down
                indicator_x = self.smart_car.x
                indicator_y = self.smart_car.y - 40  # Above car
                
                # If no other indicator is showing
                if not hasattr(self.smart_car, 'collision_risk') or not self.smart_car.collision_risk:
                    # Draw orange turn indicator
                    pygame.draw.circle(self.screen, (255, 140, 0), (int(indicator_x), int(indicator_y)), 15)
                    font = pygame.font.SysFont('Arial', 14)
                    text = font.render("TURN", True, self.BLACK)
                    self.screen.blit(text, (indicator_x - 15, indicator_y - 7))
            
            self.draw_car(self.smart_car.x, self.smart_car.y, self.smart_car.angle, collision_color)
            
            # Draw hitbox for debugging
            if hasattr(self, 'show_hitboxes') and self.show_hitboxes:
                corners = self.get_car_corners(self.smart_car.x, self.smart_car.y, self.smart_car.angle)
                # Use red hitbox when collision risk is detected
                hitbox_color = (255, 0, 0) if hasattr(self.smart_car, 'collision_risk') and self.smart_car.collision_risk else (255, 165, 0)
                pygame.draw.lines(self.screen, hitbox_color, True, corners, 2)
                
                # Draw detection zone (larger hitbox representing sensor range - now much bigger)
                detection_corners = self.get_car_corners(self.smart_car.x, self.smart_car.y, self.smart_car.angle, safety_margin=self.detection_zone_size)
                pygame.draw.lines(self.screen, (100, 100, 255), True, detection_corners, 1)
                
                # Add expanded blue detection circle
                detection_surface = pygame.Surface((self.detection_range*2, self.detection_range*2), pygame.SRCALPHA)
                pygame.draw.circle(detection_surface, (0, 0, 255, 20), (self.detection_range, self.detection_range), self.detection_range)
                self.screen.blit(detection_surface, (self.smart_car.x - self.detection_range, self.smart_car.y - self.detection_range))
                
                # If we're approaching a turn, visualize the turn angle with an arc
                if self.current_target < len(self.route) - 1:
                    current_target_pt = self.route[self.current_target]
                    next_target_pt = self.route[self.current_target + 1]
                    
                    # Draw a line to show next segment
                    pygame.draw.line(self.screen, (100, 255, 100), current_target_pt, next_target_pt, 3)
                    
                    # Calculate vectors and angle
                    dx1 = current_target_pt[0] - self.smart_car.x
                    dy1 = current_target_pt[1] - self.smart_car.y
                    dx2 = next_target_pt[0] - current_target_pt[0]
                    dy2 = next_target_pt[1] - current_target_pt[1]
                    
                    if dx1 != 0 or dy1 != 0 and dx2 != 0 or dy2 != 0:
                        # Normalize vectors
                        len1 = math.sqrt(dx1*dx1 + dy1*dy1)
                        len2 = math.sqrt(dx2*dx2 + dy2*dy2)
                        
                        if len1 > 0 and len2 > 0:
                            nx1, ny1 = dx1/len1, dy1/len1
                            nx2, ny2 = dx2/len2, dy2/len2
                            
                            # Calculate dot product
                            dot = nx1*nx2 + ny1*ny2
                            dot = max(-1.0, min(1.0, dot))  # Clamp to avoid domain errors
                            
                            # Calculate angle in degrees
                            angle = math.degrees(math.acos(dot))
                            
                            # Draw angle text near the turn
                            if angle > 20:  # Only show significant turns
                                font = pygame.font.SysFont('Arial', 16)
                                angle_text = f"{angle:.1f}°"
                                text_surface = font.render(angle_text, True, (255, 100, 0))
                                self.screen.blit(text_surface, (current_target_pt[0] + 10, current_target_pt[1] + 10))
        
        # Draw grid size dropdown menu
        grid_dropdown_rect = pygame.Rect(
            int(self.width * 0.05),
            int(self.height * 0.05),
            self.dropdown_width,
            self.dropdown_height
        )
        pygame.draw.rect(self.screen, (220, 220, 220), grid_dropdown_rect)
        pygame.draw.rect(self.screen, self.BLACK, grid_dropdown_rect, 2)
        text = self.font.render(f"Grid Size: {self.current_grid_size}x{self.current_grid_size}", True, self.BLACK)
        self.screen.blit(text, (grid_dropdown_rect.x + 10, grid_dropdown_rect.y + 5))
        
        # Draw grid size dropdown options if open
        if self.dropdown_open:
            for i, size in enumerate(self.grid_sizes):
                option_rect = pygame.Rect(
                    grid_dropdown_rect.x,
                    grid_dropdown_rect.y + (i + 1) * self.dropdown_height,
                    self.dropdown_width,
                    self.dropdown_height
                )
                pygame.draw.rect(self.screen, (200, 200, 200), option_rect)
                pygame.draw.rect(self.screen, self.BLACK, option_rect, 1)
                text = self.font.render(f"{size}x{size}", True, self.BLACK)
                self.screen.blit(text, (option_rect.x + 10, option_rect.y + 5))
        
        # Draw car count dropdown menu
        car_dropdown_rect = pygame.Rect(
            int(self.width * 0.05) + self.dropdown_width + 20,
            int(self.height * 0.05),
            self.dropdown_width,
            self.dropdown_height
        )
        pygame.draw.rect(self.screen, (220, 220, 220), car_dropdown_rect)
        pygame.draw.rect(self.screen, self.BLACK, car_dropdown_rect, 2)
        text = self.font.render(f"Car Count: {self.current_car_count}", True, self.BLACK)
        self.screen.blit(text, (car_dropdown_rect.x + 10, car_dropdown_rect.y + 5))
        
        # Draw car count dropdown options if open
        if self.car_dropdown_open:
            for i, count in enumerate(self.car_counts):
                option_rect = pygame.Rect(
                    car_dropdown_rect.x,
                    car_dropdown_rect.y + (i + 1) * self.dropdown_height,
                    self.dropdown_width,
                    self.dropdown_height
                )
                pygame.draw.rect(self.screen, (200, 200, 200), option_rect)
                pygame.draw.rect(self.screen, self.BLACK, option_rect, 1)
                text = self.font.render(f"{count}", True, self.BLACK)
                self.screen.blit(text, (option_rect.x + 10, option_rect.y + 5))
        
        # Draw instructions
        font = pygame.font.SysFont('Arial', 16)
        instructions = [
            "Traffic Simulator - Simple Visualizer",
            "Click red intersection to set start, blue for end",
            "Press 'S' to randomly select start and end points",
            "Press 'P' to check route and preventatively reroute",
            "Click dropdowns to change grid size and car count",
            "Press 'H' to toggle hitboxes for collision detection",
            "Press 'R' to force the purple car to find an alternative route",
            "Purple car avoids white cars proactively (like a flu shot)",
            "ESC to exit"
        ]
        
        for i, text in enumerate(instructions):
            text_surface = font.render(text, True, self.WHITE)
            self.screen.blit(text_surface, (20, 20 + i * 20))
            
        # Display collision avoidance status if active
        if self.smart_car and hasattr(self.smart_car, 'collision_risk') and self.smart_car.collision_risk:
            status_font = pygame.font.SysFont('Arial', 18, bold=True)
            if hasattr(self.smart_car, 'current_strategy'):
                status_text = f"Collision Avoidance Active: "
                
                if self.smart_car.current_strategy == CollisionStrategy.SLOW_DOWN:
                    status_text += "SLOWING DOWN"
                    status_color = (255, 165, 0)  # Orange
                elif self.smart_car.current_strategy == CollisionStrategy.YIELD:
                    status_text += "YIELDING"
                    status_color = (255, 0, 0)  # Red 
                elif self.smart_car.current_strategy == CollisionStrategy.REROUTE:
                    status_text += "REROUTING"
                    status_color = (0, 100, 255)  # Blue
                
                status_surface = status_font.render(status_text, True, status_color)
                self.screen.blit(status_surface, (self.width - status_surface.get_width() - 20, 20))
                
        # Display route safety score
        if self.route and self.smart_car:
            # Calculate safety and efficiency metrics
            safety_score = self.calculate_path_safety(self.route)
            efficiency_score = self.calculate_path_efficiency(self.route)
            
            # Display the scores with appropriately colored indicators
            safety_font = pygame.font.SysFont('Arial', 18, bold=True)
            
            # Set color for safety score
            if safety_score >= 80:
                safety_color = (0, 200, 0)  # Green for safe
            elif safety_score >= 50:
                safety_color = (255, 165, 0)  # Orange for caution
            else:
                safety_color = (255, 0, 0)  # Red for dangerous
            
            # Set color for efficiency score
            if efficiency_score >= 90:
                efficiency_color = (0, 200, 0)  # Green for efficient
            elif efficiency_score >= 70:
                efficiency_color = (255, 165, 0)  # Orange for somewhat efficient
            else:
                efficiency_color = (255, 0, 0)  # Red for inefficient
                
            # Display safety score
            safety_text = f"Route Safety: {safety_score:.1f}/100"
            safety_surface = safety_font.render(safety_text, True, safety_color)
            self.screen.blit(safety_surface, (self.width - safety_surface.get_width() - 20, 50))
            
            # Display efficiency score
            efficiency_text = f"Route Efficiency: {efficiency_score:.1f}%"
            efficiency_surface = safety_font.render(efficiency_text, True, efficiency_color)
            self.screen.blit(efficiency_surface, (self.width - efficiency_surface.get_width() - 20, 80))
            
            # Calculate and display the combined score (balance of safety and efficiency)
            combined_score = (safety_score * 0.6) + (efficiency_score * 0.4)  # 60% safety, 40% efficiency
            
            # Set color for combined score
            if combined_score >= 80:
                combined_color = (0, 200, 0)  # Green for good
            elif combined_score >= 60:
                combined_color = (255, 165, 0)  # Orange for average
            else:
                combined_color = (255, 0, 0)  # Red for poor
                
            combined_text = f"Overall Route Quality: {combined_score:.1f}/100"
            combined_surface = safety_font.render(combined_text, True, combined_color)
            self.screen.blit(combined_surface, (self.width - combined_surface.get_width() - 20, 110))
    
    def reset_simulation(self):
        # Reset route data
        self.start_point = None
        self.end_point = None
        self.route = []
        self.current_target = 0
        self.smart_car = None
        
        # Regenerate the grid
        self.intersections, self.roads = self.generate_grid()
        
        # Create new white cars
        self.white_cars = self.create_white_cars()
    
    def update_car_count(self):
        """Update the number of cars in the simulation"""
        self.white_cars = self.create_white_cars()
    
    def handle_click(self, pos):
        # Check for grid size dropdown menu clicks
        grid_dropdown_rect = pygame.Rect(
            int(self.width * 0.05),
            int(self.height * 0.05),
            self.dropdown_width,
            self.dropdown_height
        )
        if grid_dropdown_rect.collidepoint(pos):
            self.dropdown_open = not self.dropdown_open
            self.car_dropdown_open = False  # Close other dropdown
            return
            
        # Check for car count dropdown clicks
        car_dropdown_rect = pygame.Rect(
            int(self.width * 0.05) + self.dropdown_width + 20,
            int(self.height * 0.05),
            self.dropdown_width,
            self.dropdown_height
        )
        if car_dropdown_rect.collidepoint(pos):
            self.car_dropdown_open = not self.car_dropdown_open
            self.dropdown_open = False  # Close other dropdown
            return
            
        # If grid size dropdown is open
        if self.dropdown_open:
            for i, size in enumerate(self.grid_sizes):
                option_rect = pygame.Rect(
                    grid_dropdown_rect.x,
                    grid_dropdown_rect.y + (i + 1) * self.dropdown_height,
                    self.dropdown_width,
                    self.dropdown_height
                )
                if option_rect.collidepoint(pos):
                    if size != self.current_grid_size:
                        self.current_grid_size = size
                        # Regenerate grid and reset simulation
                        self.reset_simulation()
                    self.dropdown_open = False
                    return
        
        # If car count dropdown is open
        if self.car_dropdown_open:
            for i, count in enumerate(self.car_counts):
                option_rect = pygame.Rect(
                    car_dropdown_rect.x,
                    car_dropdown_rect.y + (i + 1) * self.dropdown_height,
                    self.dropdown_width,
                    self.dropdown_height
                )
                if option_rect.collidepoint(pos):
                    if count != self.current_car_count:
                        self.current_car_count = count
                        self.update_car_count()
                    self.car_dropdown_open = False
                    return
        
        # Check for intersection clicks
        for intersection in self.intersections:
            # Distance to intersection
            dx = pos[0] - intersection[0]
            dy = pos[1] - intersection[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            # If clicked on an intersection
            if distance < 15:
                # If no start point, set as start
                if self.start_point is None:
                    # Set start point
                    self.start_point = intersection
                    try:
                        # Create the smart car at the start point
                        print(f"Creating SmartCar at {intersection[0]}, {intersection[1]}")
                        self.smart_car = SmartCar(intersection[0], intersection[1], [], self)
                        print("Setting map data...")
                        self.smart_car.set_map_data(self.intersections, self.roads)
                        print("SmartCar created successfully")
                    except Exception as e:
                        print(f"Error creating SmartCar: {e}")
                        import traceback
                        traceback.print_exc()
                        # Create a backup simple car
                        if IMPORT_ERROR:
                            self.smart_car = None
                    self.current_target = 0
                    self.route = []
                    return
                    
                # If start point exists but no end point
                elif self.end_point is None and intersection != self.start_point:
                    # Set end point
                    self.end_point = intersection
                    
                    # Find path
                    path = self.find_path(self.start_point, self.end_point)
                    if path:
                        self.route = path
                        self.current_target = 1  # Skip start point
                        # Update the smart car's route
                        self.smart_car.route = path
                        self.smart_car.current_target = 1
                    else:
                        # No path found, reset
                        self.start_point = intersection
                        self.end_point = None
                        try:
                            # Create a new smart car at this location
                            print(f"Creating SmartCar at {intersection[0]}, {intersection[1]} (path not found)")
                            self.smart_car = SmartCar(intersection[0], intersection[1], [], self)
                            print("Setting map data...")
                            self.smart_car.set_map_data(self.intersections, self.roads)
                            print("SmartCar created successfully")
                        except Exception as e:
                            print(f"Error creating SmartCar: {e}")
                            import traceback
                            traceback.print_exc()
                            # Create a backup simple car
                            if IMPORT_ERROR:
                                self.smart_car = None
                        self.current_target = 0
                        self.route = []
                    return
                    
                # If both start and end exist, reset and set as new start
                else:
                    self.start_point = intersection
                    self.end_point = None
                    try:
                        # Create a new smart car at this location
                        print(f"Creating SmartCar at {intersection[0]}, {intersection[1]} (reset)")
                        self.smart_car = SmartCar(intersection[0], intersection[1], [], self)
                        print("Setting map data...")
                        self.smart_car.set_map_data(self.intersections, self.roads)
                        print("SmartCar created successfully")
                    except Exception as e:
                        print(f"Error creating SmartCar: {e}")
                        import traceback
                        traceback.print_exc()
                        # Create a backup simple car
                        if IMPORT_ERROR:
                            self.smart_car = None
                    self.current_target = 0
                    self.route = []
                    return
    
    def run(self):
        running = True
        frame_counter = 0
        
        try:
            while running:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            running = False
                        elif event.key == pygame.K_h:
                            # Toggle hitbox display
                            self.show_hitboxes = not self.show_hitboxes
                            print(f"Hitboxes {'on' if self.show_hitboxes else 'off'} - showing {'ghost danger zones' if self.show_hitboxes else 'normal view'}")
                        elif event.key == pygame.K_r:
                            # Force purple car to reroute (for testing)
                            if self.smart_car and self.route and len(self.route) > 1:
                                print("Forcing ghost-aware reroute!")
                                current_pos = (self.smart_car.x, self.smart_car.y)
                                end_point = self.route[-1]
                                
                                # Find a random intersection to avoid
                                if self.intersections and len(self.intersections) > 1:
                                    # Choose the next node in the current route to avoid
                                    if self.current_target < len(self.route):
                                        avoid_point = self.route[self.current_target]
                                        
                                        # Find nearest intersection to current position
                                        start_intersection = self._find_nearest_intersection(current_pos)
                                        
                                        if start_intersection:
                                            # Use ghost-aware pathfinding
                                            new_route = self.find_path_avoiding_ghosts(
                                                start_intersection, 
                                                end_point, 
                                                avoid_point
                                            )
                                            
                                            if new_route and len(new_route) > 1:
                                                safety = self.calculate_path_safety(new_route)
                                                print(f"Rerouted with {len(new_route)} points, safety: {safety}/100")
                                                self.smart_car.route = [current_pos] + new_route
                                                self.smart_car.current_target = 1
                                                self.route = self.smart_car.route
                                                self.current_target = 1
                                                # Set the strategy to REROUTE 
                                                self.smart_car.current_strategy = CollisionStrategy.REROUTE
                        elif event.key == pygame.K_p:
                            # Force preventative reroute check (for testing)
                            if self.smart_car and self.route and len(self.route) > 1:
                                print("Checking route safety and rerouting if needed...")
                                self.preventative_reroute()
                        elif event.key == pygame.K_s:
                            # Random start and finish with ghost-aware routing
                            print("Generating random start and finish points with ghost avoidance...")
                            self.random_start_finish()
                    elif event.type == pygame.MOUSEBUTTONDOWN:
                        self.handle_click(pygame.mouse.get_pos())
                
                # Update main car position
                self.update_main_car()
                
                # Update white cars
                for car in self.white_cars:
                    car.update()
                
                # Periodically validate routes (every 60 frames)
                frame_counter += 1
                if frame_counter % 60 == 0:
                    self.validate_routes()
                
                # Draw everything
                self.draw_scene()
                
                # Update the display
                pygame.display.flip()
                self.clock.tick(60)
        except Exception as e:
            self.get_logger().error(f"Error in visualization: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

    def has_road_between(self, point1, point2):
        """Check if there's a direct road between two points"""
        return (point1, point2) in self.roads or (point2, point1) in self.roads

    def validate_routes(self):
        """Validate all vehicle routes to ensure they follow roads and highlight invalid segments"""
        for car in self.white_cars:
            if len(car.route) < 2:
                continue
                
            # Check each segment
            for i in range(len(car.route) - 1):
                if not self.has_road_between(car.route[i], car.route[i+1]):
                    # Invalid segment found - print and correct
                    print(f"Invalid route segment: {car.route[i]} to {car.route[i+1]}")
                    
                    # Fix by generating a new route
                    new_route = self.generate_random_route()
                    car.route = new_route
                    car.vehicle.route = new_route
                    car.vehicle.current_target = 1
                    car.current_target = 1
                    break

    def get_car_corners(self, x, y, angle, safety_margin=1.0):
        """
        Calculate the four corners of a car for collision detection
        safety_margin: multiplier for car size (1.0 = normal size, 2.0 = twice as large)
        """
        rad_angle = math.radians(angle)
        
        # Calculate corners (unrotated)
        half_width = (self.CAR_WIDTH * safety_margin) // 2
        half_height = (self.CAR_HEIGHT * safety_margin) // 2
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
            rotated_corners.append((x + rotated_x, y + rotated_y))
            
        return rotated_corners
    
    def check_collision(self, car1, car2):
        """
        Check if two cars are colliding using Separating Axis Theorem (SAT)
        Now considers traffic lanes based on direction of travel
        """
        # First, check if cars are traveling in significantly different directions
        # If so, we can assume they're in different lanes and won't collide
        if hasattr(car1, 'angle') and hasattr(car2, 'angle'):
            angle_diff = abs((car1.angle - car2.angle + 180) % 360 - 180)
            
            # If cars are moving nearly perpendicular (75-105 degrees) 
            # or opposite (165-195 degrees), consider them in different lanes
            if (75 < angle_diff < 105) or angle_diff > 165:
                # Cars in perpendicular or opposite directions - assume different lanes
                # But still check if they're very close (crossing an intersection)
                dx = car1.x - car2.x
                dy = car1.y - car2.y
                distance = math.sqrt(dx*dx + dy*dy)
                
                # Only ignore lane-based pass-through if they're not very close
                if distance > self.CAR_WIDTH * 1.5:
                    return False
                
                # If they're at an intersection, direction doesn't save them
                # Check if they're near any intersection
                for intersection in self.intersections:
                    int_dx = car1.x - intersection[0]
                    int_dy = car1.y - intersection[1]
                    int_distance = math.sqrt(int_dx*int_dx + int_dy*int_dy)
                    
                    if int_distance < self.ROAD_WIDTH * 1.5:
                        # They're at an intersection, so lane rules don't apply
                        # Continue with normal collision detection
                        break
                else:
                    # Not at intersection and moving in different directions
                    # Assume they're in different lanes and won't collide
                    return False
        
        # Use a larger safety margin for the purple car (SmartCar) to keep it further away
        # from white cars
        purple_safety_margin = 1.5  # Increase the collision boundary by 50%
        
        # Get corners of both cars
        # If car1 is the purple car (SmartCar), use the larger safety margin
        if hasattr(car1, 'id') and car1.id == "purple_car":
            corners1 = self.get_car_corners(car1.x, car1.y, car1.angle, safety_margin=purple_safety_margin)
        else:
            corners1 = self.get_car_corners(car1.x, car1.y, car1.angle)
            
        # Always use normal hitbox for white cars
        corners2 = self.get_car_corners(car2.x, car2.y, car2.angle)
        
        # Get edges from corners
        edges = []
        for i in range(len(corners1)):
            edges.append((corners1[i], corners1[(i+1) % len(corners1)]))
        for i in range(len(corners2)):
            edges.append((corners2[i], corners2[(i+1) % len(corners2)]))
        
        # Get normal vectors for each edge
        normals = []
        for edge in edges:
            dx = edge[1][0] - edge[0][0]
            dy = edge[1][1] - edge[0][1]
            length = math.sqrt(dx*dx + dy*dy)
            if length > 0:
                # Normal vector is perpendicular to edge
                normals.append((-dy/length, dx/length))
        
        # For each normal vector, project both polygons onto it
        for normal in normals:
            # Project both polygons onto the normal
            min1, max1 = float('inf'), float('-inf')
            min2, max2 = float('inf'), float('-inf')
            
            # Project first polygon
            for corner in corners1:
                projection = corner[0] * normal[0] + corner[1] * normal[1]
                min1 = min(min1, projection)
                max1 = max(max1, projection)
            
            # Project second polygon
            for corner in corners2:
                projection = corner[0] * normal[0] + corner[1] * normal[1]
                min2 = min(min2, projection)
                max2 = max(max2, projection)
            
            # Check for gap
            if max1 < min2 or max2 < min1:
                # Found a separating axis, no collision
                return False
        
        # No separating axis found, collision detected
        return True

    def predict_collision(self, car1, car2, time_steps=15):  # Increased from 10 to 15 for longer prediction
        """
        Predict if two cars will collide within the next few time steps
        Returns: 
            - bool: True if collision predicted, False otherwise
            - float: Time to collision (0 if immediate, higher for future collisions)
        """
        # Clone car positions for prediction
        x1, y1 = car1.x, car1.y
        x2, y2 = car2.x, car2.y
        angle1, angle2 = car1.angle, car2.angle
        
        # Get velocity components
        vx1 = car1.vx if hasattr(car1, 'vx') else 0
        vy1 = car1.vy if hasattr(car1, 'vy') else 0
        vx2 = car2.vehicle.vx if hasattr(car2, 'vehicle') else 0
        vy2 = car2.vehicle.vy if hasattr(car2, 'vehicle') else 0
        
        # Check for immediate collision
        if self.check_collision(car1, car2):
            return True, 0
        
        # Check future positions
        for step in range(1, time_steps+1):
            # Update positions based on velocity
            future_x1 = x1 + vx1 * step
            future_y1 = y1 + vy1 * step
            future_x2 = x2 + vx2 * step
            future_y2 = y2 + vy2 * step
            
            # Create temporary objects to represent future positions
            class TempCar:
                def __init__(self, x, y, angle, car_id=None):
                    self.x = x
                    self.y = y
                    self.angle = angle
                    self.id = car_id
            
            # Pass the car's ID if it's the purple car
            future_car1 = TempCar(future_x1, future_y1, angle1, 
                                car_id="purple_car" if hasattr(car1, 'id') and car1.id == "purple_car" else None)
            future_car2 = TempCar(future_x2, future_y2, angle2)
            
            if self.check_collision(future_car1, future_car2):
                return True, step
        
        return False, -1
    
    def calculate_path_safety(self, path):
        """
        Calculate how safe a path is based on proximity to white cars
        Returns a safety score (higher is better)
        """
        if not path or len(path) < 2:
            return 0
            
        safety_score = 100  # Start with perfect score
        
        # Penalize for each white car near the path
        for point in path:
            for car in self.white_cars:
                dx = point[0] - car.x
                dy = point[1] - car.y
                distance = math.sqrt(dx*dx + dy*dy)
                
                # Cars very close to path points are a big penalty
                if distance < self.ROAD_WIDTH * 2:
                    safety_score -= 20
                # Cars somewhat close are a moderate penalty    
                elif distance < self.ROAD_WIDTH * 4:
                    safety_score -= 10
                # Cars in the general vicinity are a small penalty
                elif distance < self.ROAD_WIDTH * 6:
                    safety_score -= 5
                    
        return max(0, safety_score)  # Don't go below 0
    
    def calculate_path_efficiency(self, path, start_point=None, end_point=None):
        """
        Calculate how efficient a path is (based on directness and length)
        Returns a percentage (100% = optimal shortest path)
        """
        if not path or len(path) < 2:
            return 0
            
        # Use provided start/end or extract from path
        start = start_point if start_point else path[0]
        end = end_point if end_point else path[-1]
        
        # Calculate actual path length
        actual_length = 0
        for i in range(len(path) - 1):
            dx = path[i+1][0] - path[i][0]
            dy = path[i+1][1] - path[i][1]
            segment_length = math.sqrt(dx*dx + dy*dy)
            actual_length += segment_length
            
        # Calculate/estimate shortest possible path
        shortest_length = self.estimate_shortest_distance(start, end)
        if shortest_length == 0:  # Couldn't find a path, use direct distance
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            shortest_length = math.sqrt(dx*dx + dy*dy)
            
        # Calculate efficiency (closer to 100% is better)
        if shortest_length > 0:
            efficiency = min(100, (shortest_length / actual_length) * 100)
        else:
            efficiency = 100  # If both start and end are the same
            
        return efficiency
    
    def check_all_collisions(self):
        """Check for collisions between the purple car and white cars, including predicted collisions"""
        collisions = []
        predicted_collisions = []
        
        # Only check purple car against white cars
        if self.smart_car is not None:
            for car in self.white_cars:
                # Check for immediate collision
                if self.check_collision(self.smart_car, car):
                    collisions.append((self.smart_car, car))
                    
                    # Set collision risk flag
                    self.smart_car.collision_risk = True
                    # Emergency stop will be handled by gradual deceleration
                    self.smart_car.current_strategy = CollisionStrategy.YIELD
                
                # Check for predicted collisions
                else:
                    # Check if car is within detection range (bigger blue area)
                    dx = car.x - self.smart_car.x
                    dy = car.y - self.smart_car.y
                    distance = math.sqrt(dx*dx + dy*dy)
                    
                    # Increase detection range for more proactive avoidance
                    # Only consider cars within the expanded detection range
                    if distance <= self.detection_range * 1.25:  # Increase detection range by 25%
                        will_collide, time_to_collision = self.predict_collision(self.smart_car, car)
                        if will_collide:
                            predicted_collisions.append((self.smart_car, car, time_to_collision))
                            
                            # Set appropriate strategy based on time to collision
                            self.smart_car.collision_risk = True
                            
                            # Very close collision - slow down significantly
                            if time_to_collision <= 7:  # Increased from 5 to 7 for earlier reaction
                                self.smart_car.current_strategy = CollisionStrategy.SLOW_DOWN
                                
                            # Further collision - consider rerouting only if we're near an intersection
                            elif time_to_collision <= 12:  # Increased from 10 to 12 for earlier rerouting
                                # Find the nearest intersection
                                current_pos = (self.smart_car.x, self.smart_car.y)
                                nearest_intersection = self._find_nearest_intersection(current_pos)
                                
                                if nearest_intersection:
                                    # Check distance to nearest intersection
                                    dx = current_pos[0] - nearest_intersection[0]
                                    dy = current_pos[1] - nearest_intersection[1]
                                    dist_to_intersection = math.sqrt(dx*dx + dy*dy)
                                    
                                    # Only try to reroute if we're close to an intersection
                                    # Otherwise just slow down and wait until we reach one
                                    if dist_to_intersection < self.ROAD_WIDTH * 0.8:
                                        # We're close to an intersection, rerouting is reasonable
                                        if self.smart_car.current_strategy != CollisionStrategy.REROUTE:
                                            # Trigger rerouting logic
                                            self.initiate_reroute(self.smart_car, car)
                                    else:
                                        # We're in the middle of a road, just slow down
                                        self.smart_car.current_strategy = CollisionStrategy.SLOW_DOWN
                                else:
                                    # No intersection found, just slow down
                                    self.smart_car.current_strategy = CollisionStrategy.SLOW_DOWN
        
        return collisions, predicted_collisions
    
    def find_path_avoiding_ghosts(self, start, end, avoid_node=None):
        """
        Find a path that avoids white cars (like Pacman avoiding ghosts)
        Uses A* search algorithm with penalties for paths near white cars
        Balances between shortest path and safety
        """
        if start == end:
            return [start]
            
        # Build an adjacency list based on actual roads
        adjacency = {}
        for point in self.intersections:
            if point != avoid_node:  # Don't include the node to avoid if specified
                adjacency[point] = []
            
        # Add connected points based on actual roads
        for road in self.roads:
            a, b = road
            # Skip roads with avoid_node if specified
            if avoid_node and (avoid_node == a or avoid_node == b):
                continue
                
            # Add both directions
            if a in adjacency:
                adjacency[a].append(b)
            if b in adjacency:
                adjacency[b].append(a)
        
        # Check if either start or end is not in adjacency list
        if start not in adjacency or end not in adjacency:
            return None  # No path possible
            
        # Get positions of all white cars
        ghost_positions = [(car.x, car.y) for car in self.white_cars]
        
        # A* algorithm with ghost avoidance
        open_set = {start}
        closed_set = set()
        
        # Configuration parameters for balancing safety vs. efficiency
        # Higher values prioritize safety over shortest path
        GHOST_PENALTY_WEIGHT = 1.0  # Base penalty weight for ghost proximity
        MAX_PATH_LENGTH_FACTOR = 1.5  # Allow paths up to 1.5x the shortest possible
        
        # First find the shortest path (ignoring ghosts) to use as reference
        # This gives us a baseline for how long the path should be
        shortest_distance = self.estimate_shortest_distance(start, end)
        if shortest_distance == 0:
            shortest_distance = math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
            
        max_allowed_distance = shortest_distance * MAX_PATH_LENGTH_FACTOR
        
        # Heuristic function - balance between distance to goal and ghost avoidance
        def heuristic(node, goal, ghosts):
            # Base distance to goal (straight-line distance)
            dx = node[0] - goal[0]
            dy = node[1] - goal[1]
            base_cost = math.sqrt(dx*dx + dy*dy)
            
            # Add ghost avoidance cost
            ghost_cost = 0
            for ghost_x, ghost_y in ghosts:
                # Calculate distance to ghost
                g_dx = node[0] - ghost_x
                g_dy = node[1] - ghost_y
                distance = math.sqrt(g_dx*g_dx + g_dy*g_dy)
                
                # If too close to ghost, add penalty
                if distance < self.ROAD_WIDTH * 5:
                    # The closer the ghost, the higher the penalty
                    ghost_cost += (self.ROAD_WIDTH * 5 - distance) * GHOST_PENALTY_WEIGHT
                    
            return base_cost + ghost_cost
        
        # Costs and paths
        g_score = {start: 0}  # Cost from start to node
        f_score = {start: heuristic(start, end, ghost_positions)}  # Estimated total cost
        came_from = {}  # Parent nodes
        real_distance = {start: 0}  # Track actual path distance
        
        while open_set:
            # Find node with lowest f_score
            current = min(open_set, key=lambda node: f_score.get(node, float('inf')))
            
            # Check if we've reached the goal
            if current == end:
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return list(reversed(path))
                
            # Move current from open to closed
            open_set.remove(current)
            closed_set.add(current)
            
            # Explore neighbors
            for neighbor in adjacency.get(current, []):
                if neighbor in closed_set:
                    continue
                    
                # Calculate actual distance traveled
                distance = math.sqrt((current[0] - neighbor[0])**2 + (current[1] - neighbor[1])**2)
                tentative_real_distance = real_distance[current] + distance
                
                # Skip paths that are too long compared to the shortest possible
                if tentative_real_distance > max_allowed_distance:
                    continue
                
                # Calculate tentative g_score (path cost)
                tentative_g = g_score[current] + distance
                
                # Extra cost for neighbors near ghosts - make paths near ghosts more expensive
                ghost_penalty = 0
                for ghost_x, ghost_y in ghost_positions:
                    g_dx = neighbor[0] - ghost_x
                    g_dy = neighbor[1] - ghost_y
                    ghost_distance = math.sqrt(g_dx*g_dx + g_dy*g_dy)
                    
                    # Add penalty for proximity to ghosts
                    if ghost_distance < self.ROAD_WIDTH * 5:
                        # The closer the ghost, the higher the penalty
                        # Scale the penalty by distance: closer is much worse
                        proximity_factor = (self.ROAD_WIDTH * 5 - ghost_distance) / (self.ROAD_WIDTH * 5)
                        ghost_penalty += proximity_factor * GHOST_PENALTY_WEIGHT * distance * 3.0
                
                tentative_g += ghost_penalty
                
                # If neighbor not in open set, add it
                if neighbor not in open_set:
                    open_set.add(neighbor)
                # If this path to neighbor is worse than previous one, skip
                elif tentative_g >= g_score.get(neighbor, float('inf')):
                    continue
                    
                # This path is best so far - record it
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                real_distance[neighbor] = tentative_real_distance
                f_score[neighbor] = tentative_g + heuristic(neighbor, end, ghost_positions)
                
        # No path found
        return None
        
    def estimate_shortest_distance(self, start, end):
        """
        Calculate the shortest possible distance from start to end
        using only the road network (no ghost avoidance)
        """
        if start == end:
            return 0
            
        # If there's a direct road, return the straight-line distance
        if self.has_road_between(start, end):
            return math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
            
        # Build adjacency list from road network
        adjacency = {}
        for point in self.intersections:
            adjacency[point] = []
            
        for road in self.roads:
            a, b = road
            if a in adjacency:
                adjacency[a].append(b)
            if b in adjacency:
                adjacency[b].append(a)
        
        # Simple Dijkstra's algorithm to find shortest path
        dist = {start: 0}
        queue = [(0, start)]  # (distance, node)
        visited = set()
        
        while queue:
            (d, current) = min(queue)
            queue.remove((d, current))
            
            if current in visited:
                continue
                
            visited.add(current)
            
            if current == end:
                return d
                
            for neighbor in adjacency.get(current, []):
                if neighbor in visited:
                    continue
                    
                distance = math.sqrt((current[0] - neighbor[0])**2 + (current[1] - neighbor[1])**2)
                new_dist = dist[current] + distance
                
                if neighbor not in dist or new_dist < dist[neighbor]:
                    dist[neighbor] = new_dist
                    queue.append((new_dist, neighbor))
        
        # If no path found, return a large value
        return float('inf')

    def initiate_reroute(self, smart_car, obstacle_car):
        """
        Initiate rerouting to avoid a potential collision, balancing safety and efficiency
        """
        if not self.route or self.current_target >= len(self.route):
            return False
            
        current_pos = (smart_car.x, smart_car.y)
        
        # Find nearest intersection to current position
        nearest_intersection = self._find_nearest_intersection(current_pos)
        if not nearest_intersection:
            return False
            
        # Check if we're close enough to an intersection to reroute
        # Only reroute if we're at or very near an intersection
        dx = current_pos[0] - nearest_intersection[0]
        dy = current_pos[1] - nearest_intersection[1]
        distance_to_intersection = math.sqrt(dx*dx + dy*dy)
        
        # If we're not near an intersection, don't reroute - just slow down
        if distance_to_intersection > self.ROAD_WIDTH * 0.8:
            # We're in the middle of a road, not at an intersection
            # Just slow down instead of rerouting
            smart_car.current_strategy = CollisionStrategy.SLOW_DOWN
            return False
            
        # We're at an intersection, so we can reroute
        current_target = smart_car.current_target
        
        if current_target < len(self.route):
            # Get next waypoint to avoid
            next_waypoint = self.route[current_target]
            node_to_avoid = next_waypoint
            
            # Make sure we're not trying to avoid the current intersection
            if node_to_avoid == nearest_intersection:
                # We're at the intersection we're trying to avoid
                # Move to the next target if possible, otherwise just slow down
                if current_target + 1 < len(self.route):
                    node_to_avoid = self.route[current_target + 1]
                else:
                    # No more waypoints, just slow down
                    smart_car.current_strategy = CollisionStrategy.SLOW_DOWN
                    return False
            
            print(f"Initiating balanced reroute at intersection to avoid node {node_to_avoid}")
            end_point = self.route[-1]
            
            # Use the current intersection as the starting point for rerouting
            start_intersection = nearest_intersection
            
            # Check if the next segment follows the road network
            # (prevent suggesting a route that cuts across non-road areas)
            if current_target + 1 < len(self.route):
                next_point = self.route[current_target + 1]
                if not self.has_road_between(nearest_intersection, next_point):
                    # No direct road to next point, find a valid route
                    print("No direct road to next waypoint, finding valid route")
            
            # Calculate metrics for current route
            current_route = self.route[self.current_target:]
            current_safety = self.calculate_path_safety(current_route)
            current_efficiency = self.calculate_path_efficiency(current_route, current_pos, end_point)
            current_score = (current_safety * 0.6) + (current_efficiency * 0.4)
            
            # Use our enhanced pathfinding to balance safety and efficiency
            new_route = self.find_path_avoiding_ghosts(start_intersection, end_point, node_to_avoid)
            
            if new_route and len(new_route) > 1:
                # Calculate metrics for new route
                new_safety = self.calculate_path_safety(new_route)
                new_efficiency = self.calculate_path_efficiency(new_route, start_intersection, end_point)
                new_score = (new_safety * 0.6) + (new_efficiency * 0.4)
                
                print(f"Route comparison - Current: Safety {current_safety:.1f}/100, Efficiency {current_efficiency:.1f}%, " +
                      f"Overall {current_score:.1f}/100")
                print(f"Route comparison - New: Safety {new_safety:.1f}/100, Efficiency {new_efficiency:.1f}%, " +
                      f"Overall {new_score:.1f}/100")
                
                # Only reroute if the new route is significantly better
                if new_score > current_score + 5:  # Require at least 5 point improvement
                    # Check if this would cause a U-turn
                    is_uturn = False
                    
                    # If we have a current direction, see if the new route turns us back
                    if hasattr(smart_car, 'angle'):
                        # Get the direction to first point in new route
                        if len(new_route) > 1:
                            new_dx = new_route[1][0] - start_intersection[0] 
                            new_dy = new_route[1][1] - start_intersection[1]
                            new_angle = math.degrees(math.atan2(new_dy, new_dx))
                            
                            # Calculate angle difference
                            angle_diff = abs((new_angle - smart_car.angle + 180) % 360 - 180)
                            
                            # If the angle difference is too large, it's a U-turn
                            if angle_diff > 135:  # More than 135 degrees is a U-turn
                                is_uturn = True
                                print(f"Rejecting reroute: Would cause U-turn (angle diff: {angle_diff:.1f}°)")
                    
                    if not is_uturn:
                        print(f"Found better route! Overall score: {current_score:.1f} → {new_score:.1f}/100")
                        
                        # Replace route from current position onwards
                        smart_car.route = [current_pos] + new_route
                        smart_car.current_target = 1  # Start from next point
                        # Update visualizer's route
                        self.route = smart_car.route
                        self.current_target = 1
                        # Set the strategy to REROUTE
                        smart_car.current_strategy = CollisionStrategy.REROUTE
                        return True
                    else:
                        # It's a U-turn, so just slow down instead
                        smart_car.current_strategy = CollisionStrategy.SLOW_DOWN
                else:
                    print(f"Rejecting reroute: Not significantly better ({new_score:.1f} ≤ {current_score + 5:.1f})")
                    # Route is not better enough, just slow down
                    smart_car.current_strategy = CollisionStrategy.SLOW_DOWN
        
        return False

    def _find_nearest_intersection(self, point):
        """Find the nearest intersection to a given point"""
        closest_dist = float('inf')
        closest_point = None
        
        for intersection in self.intersections:
            dx = point[0] - intersection[0]
            dy = point[1] - intersection[1]
            dist = math.sqrt(dx*dx + dy*dy)
            
            if dist < closest_dist:
                closest_dist = dist
                closest_point = intersection
                
        return closest_point
    
    def random_start_finish(self):
        """Randomly select start and finish points and generate a route"""
        if not self.intersections or len(self.intersections) < 2:
            print("Not enough intersections to create a random route")
            return False
            
        # Clear existing route
        self.start_point = None
        self.end_point = None
        self.route = []
        self.current_target = 0
        
        # Select random start point
        self.start_point = random.choice(self.intersections)
        
        # Select random end point (different from start)
        available_ends = [p for p in self.intersections if p != self.start_point]
        if not available_ends:
            print("No valid end points available")
            return False
            
        self.end_point = random.choice(available_ends)
        
        print(f"Selected random start: {self.start_point}, end: {self.end_point}")
        
        # Create the smart car at the start point
        try:
            self.smart_car = SmartCar(self.start_point[0], self.start_point[1], [], self)
            self.smart_car.set_map_data(self.intersections, self.roads)
            
            # Find ghost-aware path between points
            path = self.find_path_avoiding_ghosts(self.start_point, self.end_point)
            if path:
                safety = self.calculate_path_safety(path)
                print(f"Found ghost-aware route with {len(path)} points (safety: {safety}/100)")
                self.route = path
                self.current_target = 1  # Skip start point
                # Update the smart car's route
                self.smart_car.route = path
                self.smart_car.original_route = path.copy()
                self.smart_car.current_target = 1
                return True
            else:
                # Fall back to regular pathfinding if ghost-aware fails
                print("Ghost-aware routing failed, falling back to regular routing")
                path = self.find_path(self.start_point, self.end_point)
                if path:
                    self.route = path
                    self.current_target = 1  # Skip start point
                    # Update the smart car's route
                    self.smart_car.route = path
                    self.smart_car.original_route = path.copy()
                    self.smart_car.current_target = 1
                    print(f"Found fallback route with {len(path)} points")
                    return True
                else:
                    print("No valid path found between random points")
                    return False
                
        except Exception as e:
            print(f"Error creating SmartCar with random route: {e}")
            import traceback
            traceback.print_exc()
            return False

    def check_route_for_conflicts(self):
        """
        Preventatively check entire route for potential conflicts with white cars
        Returns: 
            - bool: True if conflicts found, False if route is clear
            - int: Index of first conflicted waypoint
            - float: Safety score of the route (0-100)
            - float: Efficiency score of the route (0-100)
            - float: Combined route quality score (0-100)
        """
        if not self.route or not self.smart_car or len(self.route) <= self.current_target:
            return False, -1, 100, 100, 100  # No conflicts possible
            
        # Get current position
        current_pos = (self.smart_car.x, self.smart_car.y)
        
        # Get remaining route waypoints
        remaining_route = self.route[self.current_target:]
        
        # Check each waypoint along the route for nearby white cars
        conflicts = []
        for i, waypoint in enumerate(remaining_route):
            # Check distance to all white cars
            for car in self.white_cars:
                dx = waypoint[0] - car.x
                dy = waypoint[1] - car.y
                distance = math.sqrt(dx*dx + dy*dy)
                
                # Calculate relative velocity to estimate future position
                rel_vx = 0
                rel_vy = 0
                if hasattr(car, 'vehicle') and hasattr(car.vehicle, 'vx') and hasattr(car.vehicle, 'vy'):
                    rel_vx = car.vehicle.vx
                    rel_vy = car.vehicle.vy
                
                # Estimated steps to reach this waypoint from current position
                if i == 0:
                    dx_to_waypoint = waypoint[0] - current_pos[0]
                    dy_to_waypoint = waypoint[1] - current_pos[1]
                else:
                    dx_to_waypoint = waypoint[0] - remaining_route[i-1][0]
                    dy_to_waypoint = waypoint[1] - remaining_route[i-1][1]
                
                dist_to_waypoint = math.sqrt(dx_to_waypoint*dx_to_waypoint + dy_to_waypoint*dy_to_waypoint)
                steps_to_waypoint = dist_to_waypoint / self.smart_car.speed if self.smart_car.speed > 0 else 100
                
                # Predict where the white car will be when we reach this waypoint
                future_car_x = car.x + rel_vx * steps_to_waypoint
                future_car_y = car.y + rel_vy * steps_to_waypoint
                
                # Calculate distance to predicted future position
                future_dx = waypoint[0] - future_car_x
                future_dy = waypoint[1] - future_car_y
                future_distance = math.sqrt(future_dx*future_dx + future_dy*future_dy)
                
                # Check if future position creates a conflict
                conflict_threshold = self.ROAD_WIDTH * 3
                if future_distance < conflict_threshold:
                    conflicts.append((i, waypoint, car, future_distance))
        
        # Calculate route safety score (100 = completely safe, 0 = very dangerous)
        safety_score = self.calculate_path_safety(self.route)
        
        # Calculate route efficiency score
        efficiency_score = self.calculate_path_efficiency(self.route)
        
        # Calculate combined score (weighted balance of safety and efficiency)
        combined_score = (safety_score * 0.6) + (efficiency_score * 0.4)
        
        # Return conflict status and scores
        if conflicts:
            # Return index of first conflicted waypoint and scores
            first_conflict = min(conflicts, key=lambda x: x[0])
            return True, first_conflict[0] + self.current_target, safety_score, efficiency_score, combined_score
        
        return False, -1, safety_score, efficiency_score, combined_score
    
    def preventative_reroute(self):
        """
        Proactively reroute to avoid white cars in the planned route
        Balances safety (avoiding cars) with efficiency (shorter paths)
        """
        # Check if route needs to be checked
        has_conflicts, conflict_index, safety_score, efficiency_score, combined_score = self.check_route_for_conflicts()
        
        # Print detailed route metrics
        print(f"Route metrics - Safety: {safety_score:.1f}/100, Efficiency: {efficiency_score:.1f}%, " +
              f"Overall: {combined_score:.1f}/100")
        
        # Threshold for when to consider rerouting
        # Now uses combined score rather than just safety
        quality_threshold = self.route_safety_threshold
        
        # If route is good enough (based on combined score), do nothing
        if combined_score >= quality_threshold:
            return False
            
        # If we have conflicts, try to find a better route
        if has_conflicts and conflict_index >= 0:
            print(f"Preventative rerouting: Conflict detected at waypoint {conflict_index}")
            
            # Get current position and end goal
            current_pos = (self.smart_car.x, self.smart_car.y)
            end_point = self.route[-1]
            
            # Find nearest intersection to current position
            nearest_intersection = self._find_nearest_intersection(current_pos)
            
            # Check if we're close to an intersection
            if nearest_intersection:
                dx = current_pos[0] - nearest_intersection[0]
                dy = current_pos[1] - nearest_intersection[1]
                dist_to_intersection = math.sqrt(dx*dx + dy*dy)
                
                # Only reroute at or near intersections
                if dist_to_intersection > self.ROAD_WIDTH * 0.8:
                    print("Not at intersection, skipping preventative reroute")
                    return False
            else:
                return False
            
            # Get the conflict point to avoid
            conflict_point = self.route[conflict_index]
            
            # Find a ghost-aware route avoiding the conflict point
            new_route = self.find_path_avoiding_ghosts(nearest_intersection, end_point, conflict_point)
            
            if new_route and len(new_route) > 1:
                # Calculate safety, efficiency and combined score for new route
                new_safety = self.calculate_path_safety(new_route)
                new_efficiency = self.calculate_path_efficiency(new_route, nearest_intersection, end_point)
                new_combined = (new_safety * 0.6) + (new_efficiency * 0.4)
                
                print(f"New route metrics - Safety: {new_safety:.1f}/100, Efficiency: {new_efficiency:.1f}%, " +
                      f"Overall: {new_combined:.1f}/100")
                
                # Only use new route if the combined score is better
                if new_combined > combined_score + 5:  # Require at least 5 point improvement
                    # Check if this would cause a U-turn
                    is_uturn = False
                    
                    # If we have a current direction, see if the new route turns us back
                    if hasattr(self.smart_car, 'angle'):
                        # Get the direction to first point in new route
                        if len(new_route) > 1:
                            new_dx = new_route[1][0] - nearest_intersection[0] 
                            new_dy = new_route[1][1] - nearest_intersection[1]
                            new_angle = math.degrees(math.atan2(new_dy, new_dx))
                            
                            # Calculate angle difference
                            angle_diff = abs((new_angle - self.smart_car.angle + 180) % 360 - 180)
                            
                            # If the angle difference is too large, it's a U-turn
                            if angle_diff > 135:  # More than 135 degrees is a U-turn
                                is_uturn = True
                                print(f"Rejecting preventative reroute: Would cause U-turn (angle diff: {angle_diff:.1f}°)")
                    
                    if not is_uturn:
                        print(f"Preventative reroute: Found better route (Overall: {combined_score:.1f} → {new_combined:.1f}/100)")
                        self.smart_car.route = [current_pos] + new_route
                        self.smart_car.current_target = 1
                        self.route = self.smart_car.route
                        self.current_target = 1
                        self.smart_car.current_strategy = CollisionStrategy.REROUTE
                        return True
                else:
                    print(f"Preventative reroute: New route not better overall ({new_combined:.1f} ≤ {combined_score + 5:.1f})")
        
        return False


class WhiteCar:
    """
    Compatibility wrapper for the WhiteCar class in the vehicle module
    """
    def __init__(self, x, y, route, node=None):
        # Create the actual vehicle object from our module
        from traffic_simulation.vehicles.vehicle import WhiteCar as RealWhiteCar
        self.vehicle = RealWhiteCar(x, y, route, node)
        self.x = x
        self.y = y
        self.angle = 0
        self.route = route
        self.current_target = 1
        self.width = 60
        self.height = 30
        self.node = node  # Store reference to visualizer
    
    def update(self):
        try:
            # Update the actual vehicle with the current list of cars
            self.vehicle.update([])
            
            # Update the local attributes for visualization
            self.x = self.vehicle.x
            self.y = self.vehicle.y
            self.angle = self.vehicle.angle
            self.current_target = self.vehicle.current_target
            
            # Check if the car has finished its route and needs a new one
            if self.current_target >= len(self.route) and self.node:
                # Find the closest intersection to the car's current position
                closest_intersection = None
                closest_dist = float('inf')
                
                for intersection in self.node.intersections:
                    dx = self.x - intersection[0]
                    dy = self.y - intersection[1]
                    dist = math.sqrt(dx*dx + dy*dy)
                    
                    if dist < closest_dist:
                        closest_dist = dist
                        closest_intersection = intersection
                
                # Generate a route starting from the closest intersection
                # Try to find a path to a random destination
                start = closest_intersection
                attempts = 0
                max_attempts = min(15, len(self.node.intersections))
                valid_route = False
                
                while attempts < max_attempts and not valid_route:
                    attempts += 1
                    end = random.choice(self.node.intersections)
                    
                    # Skip if same point
                    if start == end:
                        continue
                    
                    # Find path
                    new_route = self.node.find_path(start, end)
                    
                    # Check if path is valid and has at least two points
                    if new_route and len(new_route) >= 2:
                        # Verify each segment is a valid road
                        valid_route = True
                        for i in range(len(new_route) - 1):
                            if not self.node.has_road_between(new_route[i], new_route[i+1]):
                                valid_route = False
                                break
                        
                        if valid_route:
                            self.route = new_route
                            self.vehicle.route = new_route
                            self.vehicle.current_target = 1  # Skip the start point
                            self.current_target = 1
                
                # If we still don't have a valid route, use a fallback
                if not valid_route:
                    print("Couldn't find valid route from current position - using fallback")
                    # Find an adjacent intersection via a road
                    for road in self.node.roads:
                        if closest_intersection in road:
                            # Use this road as route
                            other_end = road[1] if road[0] == closest_intersection else road[0]
                            self.route = [closest_intersection, other_end]
                            self.vehicle.route = self.route
                            self.vehicle.current_target = 1
                            self.current_target = 1
                            break
        except Exception as e:
            print(f"Error updating WhiteCar: {e}")


def main(args=None):
    try:
        print("Initializing ROS2...")
        rclpy.init(args=args)
        
        print("Creating SimpleVisualizer node...")
        node = SimpleVisualizer()
        
        print("Starting SimpleVisualizer run loop...")
        node.run()
        
        print("SimpleVisualizer run completed normally")
    
    except KeyboardInterrupt:
        print("Keyboard interrupt received, shutting down")
    
    except Exception as e:
        print(f"Error in main: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        print("Cleaning up...")
        try:
            if 'node' in locals():
                node.destroy_node()
            rclpy.shutdown()
            pygame.quit()
            print("Cleanup complete")
        except Exception as e:
            print(f"Error during cleanup: {e}")


if __name__ == '__main__':
    main() 