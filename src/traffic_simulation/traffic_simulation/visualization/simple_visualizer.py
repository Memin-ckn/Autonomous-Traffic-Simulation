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
            
            # Check for collisions with white cars before moving
            collisions, predicted_collisions = self.check_all_collisions()
            
            if collisions:
                print(f"Immediate collision detected with {len(collisions)} cars!")
            
            if predicted_collisions:
                for _, _, time in predicted_collisions:
                    print(f"Collision predicted in {time} steps")
            
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
                        
                        # Try to reroute
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
        
        # Draw white cars
        for car in self.white_cars:
            self.draw_car(car.x, car.y, car.angle, self.WHITE)
            # Draw hitbox for debugging
            if hasattr(self, 'show_hitboxes') and self.show_hitboxes:
                corners = self.get_car_corners(car.x, car.y, car.angle)
                pygame.draw.lines(self.screen, (255, 165, 0), True, corners, 2)
                
                # Draw safety zone around white cars (slightly larger hitbox)
                if self.show_hitboxes and self.smart_car is not None:
                    safety_corners = self.get_car_corners(car.x, car.y, car.angle, safety_margin=1.3)
                    pygame.draw.lines(self.screen, (255, 100, 0), True, safety_corners, 1)
        
        # Draw main car (purple)
        if self.smart_car is not None:
            collision_color = self.PURPLE
            
            # Change car color based on collision status and strategy
            if hasattr(self.smart_car, 'collision_risk') and self.smart_car.collision_risk:
                # Red tint for collision risk
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
                        
            self.draw_car(self.smart_car.x, self.smart_car.y, self.smart_car.angle, collision_color)
            
            # Draw hitbox for debugging
            if hasattr(self, 'show_hitboxes') and self.show_hitboxes:
                corners = self.get_car_corners(self.smart_car.x, self.smart_car.y, self.smart_car.angle)
                # Use red hitbox when collision risk is detected
                hitbox_color = (255, 0, 0) if hasattr(self.smart_car, 'collision_risk') and self.smart_car.collision_risk else (255, 165, 0)
                pygame.draw.lines(self.screen, hitbox_color, True, corners, 2)
                
                # Draw detection zone (larger hitbox representing sensor range)
                detection_corners = self.get_car_corners(self.smart_car.x, self.smart_car.y, self.smart_car.angle, safety_margin=2.0)
                pygame.draw.lines(self.screen, (100, 100, 255), True, detection_corners, 1)
        
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
            "Click dropdowns to change grid size and car count",
            "Press 'H' to toggle hitboxes for collision detection",
            "Press 'R' to force the purple car to find an alternative route",
            "Purple car will auto-avoid collisions using: SLOW/STOP/REROUTE",
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
                        elif event.key == pygame.K_r:
                            # Force purple car to reroute (for testing)
                            if self.smart_car and self.route and len(self.route) > 1:
                                print("Forcing reroute!")
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
                                            # Find new route avoiding the next waypoint
                                            new_route = self.find_path_avoiding_node(
                                                start_intersection, 
                                                end_point, 
                                                avoid_point
                                            )
                                            
                                            if new_route and len(new_route) > 1:
                                                print(f"Rerouted with {len(new_route)} points, avoiding {avoid_point}")
                                                self.smart_car.route = [current_pos] + new_route
                                                self.smart_car.current_target = 1
                                                self.route = self.smart_car.route
                                                self.current_target = 1
                                                # Set the strategy to REROUTE 
                                                self.smart_car.current_strategy = CollisionStrategy.REROUTE
                        elif event.key == pygame.K_s:
                            # Random start and finish
                            print("Generating random start and finish points...")
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
        """Check if two cars are colliding using Separating Axis Theorem (SAT)"""
        # Get corners of both cars
        corners1 = self.get_car_corners(car1.x, car1.y, car1.angle)
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
    
    def predict_collision(self, car1, car2, time_steps=10):
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
                def __init__(self, x, y, angle):
                    self.x = x
                    self.y = y
                    self.angle = angle
            
            future_car1 = TempCar(future_x1, future_y1, angle1)
            future_car2 = TempCar(future_x2, future_y2, angle2)
            
            if self.check_collision(future_car1, future_car2):
                return True, step
        
        return False, -1
    
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
                    # Emergency stop for immediate collisions
                    self.smart_car.speed = 0
                    self.smart_car.current_strategy = CollisionStrategy.YIELD
                
                # Check for predicted collisions
                else:
                    will_collide, time_to_collision = self.predict_collision(self.smart_car, car)
                    if will_collide:
                        predicted_collisions.append((self.smart_car, car, time_to_collision))
                        
                        # Set appropriate strategy based on time to collision
                        self.smart_car.collision_risk = True
                        
                        # Very close collision - slow down significantly or stop
                        if time_to_collision <= 3:
                            self.smart_car.current_strategy = CollisionStrategy.SLOW_DOWN
                            # Slow down more for closer collisions
                            slowdown_factor = max(0.1, time_to_collision / 5.0)
                            self.smart_car.speed = self.smart_car.default_speed * slowdown_factor
                            
                        # Further collision - try to reroute
                        elif time_to_collision <= 7:
                            # Only reroute if we weren't already doing so
                            if self.smart_car.current_strategy != CollisionStrategy.REROUTE:
                                # Trigger rerouting logic
                                self.initiate_reroute(self.smart_car, car)
        
        return collisions, predicted_collisions
    
    def initiate_reroute(self, smart_car, obstacle_car):
        """Initiate rerouting to avoid a potential collision"""
        if not self.route or self.current_target >= len(self.route):
            return
            
        current_pos = (smart_car.x, smart_car.y)
        current_target = smart_car.current_target
        
        if current_target < len(self.route):
            # Get next waypoint to avoid
            next_waypoint = self.route[current_target]
            node_to_avoid = next_waypoint
            
            print(f"Initiating preemptive reroute to avoid node {node_to_avoid}")
            end_point = self.route[-1]
            
            # Find nearest intersection to current position
            start_intersection = self._find_nearest_intersection(current_pos)
            
            if start_intersection:
                # Use our pathfinding to avoid the node
                new_route = self.find_path_avoiding_node(start_intersection, end_point, node_to_avoid)
                
                if new_route and len(new_route) > 1:
                    print(f"Found preemptive alternative route with {len(new_route)} points")
                    # Replace route from current position onwards
                    smart_car.route = [current_pos] + new_route
                    smart_car.current_target = 1  # Start from next point
                    # Update visualizer's route
                    self.route = smart_car.route
                    self.current_target = 1
                    # Set the strategy to REROUTE
                    smart_car.current_strategy = CollisionStrategy.REROUTE
                    return True
        
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
    
    def find_path_avoiding_node(self, start, end, avoid_node):
        """Find a path from start to end that avoids a specific node"""
        if start == end:
            return [start]
            
        # Build an adjacency list based on actual roads, excluding those with avoid_node
        adjacency = {}
        for point in self.intersections:
            if point != avoid_node:  # Don't include the node to avoid
                adjacency[point] = []
            
        # Add connected points based on actual roads
        for road in self.roads:
            a, b = road
            # Skip roads with avoid_node
            if avoid_node in road:
                continue
                
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
                return path
                
            if current in visited:
                continue
                
            visited.add(current)
            
            # Add neighbors from adjacency list
            for neighbor in adjacency.get(current, []):
                if neighbor not in visited:
                    queue.append((neighbor, path + [neighbor]))
        
        return None  # No path found

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
            
            # Find path between points
            path = self.find_path(self.start_point, self.end_point)
            if path:
                self.route = path
                self.current_target = 1  # Skip start point
                # Update the smart car's route
                self.smart_car.route = path
                self.smart_car.original_route = path.copy()
                self.smart_car.current_target = 1
                print(f"Found valid route with {len(path)} points")
                return True
            else:
                print("No valid path found between random points")
                return False
                
        except Exception as e:
            print(f"Error creating SmartCar with random route: {e}")
            import traceback
            traceback.print_exc()
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