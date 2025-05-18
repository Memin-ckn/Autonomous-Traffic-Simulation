#!/usr/bin/env python3

"""
Simple Visualizer for Traffic Simulation.
Main ROS2 node that implements the traffic simulation visualization using pygame.
This version is updated to use our new component-based architecture while
maintaining backward compatibility with the original simple_visualizer.py.
"""

import pygame
import sys
import math
import random
import rclpy
from rclpy.node import Node

# Import our new components
from traffic_simulation.visualization.components.renderer import SimulationRenderer
from traffic_simulation.visualization.components.world_model import WorldModel
from traffic_simulation.visualization.components.collision_detector import CollisionDetector
from traffic_simulation.visualization.components.ui_manager import UIManager

# Import vehicle classes 
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
    IMPORT_ERROR = False
except Exception as e:
    print(f"Import error: {e}")
    import traceback
    traceback.print_exc()
    # Don't crash, but set flags to avoid using these features
    IMPORT_ERROR = True


class SimpleVisualizer(Node):
    """
    Main visualization node for the traffic simulation.
    This updated version uses our new component-based architecture while
    maintaining backward compatibility with the original simple_visualizer.py.
    """
    def __init__(self):
        """Initialize the visualization node."""
        super().__init__('simple_visualizer')
        pygame.init()
        
        # Create a fullscreen window
        self.screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
        self.width, self.height = self.screen.get_size()
        pygame.display.set_caption("Traffic Simulator")
        
        # Initialize components
        self.initialize_components()
        
        # Car properties
        self.CAR_WIDTH = 60
        self.CAR_HEIGHT = 30
        
        # Main car
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
        self.detection_range = self.ROAD_WIDTH * 12  # Increased detection zone
        self.safety_zone_size = 2.0  # Safety zone multiplier
        self.detection_zone_size = 4.0  # Detection zone size
        
        # Movement smoothing parameters
        self.acceleration_rate = 0.05  # How quickly cars accelerate
        self.deceleration_rate = 0.1   # Deceleration rate
        self.min_speed = 0.1  # Minimum speed when slowing down
        
        # Route planning parameters
        self.route_check_interval = 30  # Check route every 30 frames
        self.preventative_reroute_enabled = True  # Enable preventative rerouting
        self.route_safety_threshold = 50  # Minimum safety score to avoid rerouting
        self.route_check_count = 0  # Counter for checking route
        
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
        
        # Generate the map
        self.intersections, self.roads = self.generate_grid()
        
        # Create white cars
        self.white_cars = self.create_white_cars()
        
        # For FPS control
        self.clock = pygame.time.Clock()
    
    def initialize_components(self):
        """Initialize all components for the visualization."""
        # Renderer for drawing
        self.renderer = SimulationRenderer(self.screen, self.width, self.height)
        
        # World model for roads and pathfinding
        self.world_model = WorldModel(self.width, self.height, grid_size=4)
        
        # UI manager
        self.ui_manager = UIManager(self.width, self.height)
        
        # Collision detector (will be initialized after CAR_WIDTH and ROAD_WIDTH are defined)
        self.collision_detector = None
    
    def generate_grid(self):
        """
        Generate a grid of intersections and roads.
        This is a wrapper for the WorldModel.generate_grid method for backwards compatibility.
        
        Returns:
            tuple: (intersections, roads) lists
        """
        # Initialize collision detector now that we have the road width
        self.collision_detector = CollisionDetector(self.CAR_WIDTH, self.CAR_HEIGHT, self.ROAD_WIDTH)
        
        # Use world model to generate grid
        return self.world_model.generate_grid()
    
    def has_road_between(self, point1, point2):
        """
        Check if there's a direct road between two points.
        Wrapper for WorldModel.has_road_between for backwards compatibility.
        
        Args:
            point1 (tuple): First point (x, y)
            point2 (tuple): Second point (x, y)
            
        Returns:
            bool: True if a road exists, False otherwise
        """
        return self.world_model.has_road_between(point1, point2)
    
    def find_path(self, start, end):
        """
        Find a path from start to end using only existing roads.
        Wrapper for WorldModel.find_path for backwards compatibility.
        
        Args:
            start (tuple): Start point (x, y)
            end (tuple): End point (x, y)
            
        Returns:
            list: List of points forming the path, or None if no path exists
        """
        return self.world_model.find_path(start, end)
    
    def create_white_cars(self, count=None):
        """
        Create white cars with valid routes
        
        Args:
            count (int): Number of cars to create (optional)
            
        Returns:
            list: List of WhiteCar objects
        """
        # Use the current_car_count if count is not specified
        if count is None:
            count = self.ui_manager.get_car_count()
            
        cars = []
        for _ in range(count):
            route = self.world_model.generate_random_route()
            
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
    
    def draw_car(self, x, y, angle, color):
        """
        Draw a car at the specified position and orientation.
        Wrapper for renderer.draw_car for backwards compatibility.
        
        Args:
            x (float): X coordinate
            y (float): Y coordinate
            angle (float): Angle in degrees
            color (tuple): RGB color tuple
        """
        self.renderer.draw_car(x, y, angle, color, self.CAR_WIDTH, self.CAR_HEIGHT)
    
    def update_main_car(self):
        """Update the main car position and handle collision avoidance."""
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
            
            # Periodically check if we need preventative rerouting
            self.route_check_count += 1
            if self.preventative_reroute_enabled and self.route_check_count >= self.route_check_interval:
                self.route_check_count = 0
                is_rerouted = self.preventative_reroute()
                if is_rerouted:
                    # Update our target tracking to match the smart car
                    self.current_target = self.smart_car.current_target
                    return
            
            # Pre-check for potential threats - white cars in vicinity
            for car in self.white_cars:
                dx = car.x - self.smart_car.x
                dy = car.y - self.smart_car.y
                distance = math.sqrt(dx*dx + dy*dy)
                
                # Just being in the vicinity will cause some caution
                if distance <= self.ROAD_WIDTH * 6:
                    self.smart_car.collision_risk = True
                    # Set initial strategy of slowing down due to proximity
                    if not hasattr(self.smart_car, 'current_strategy') or self.smart_car.current_strategy is None:
                        self.smart_car.current_strategy = self.CollisionStrategy.SLOW_DOWN
            
            # Check for collisions
            collisions, predicted_collisions = self.check_all_collisions()
            
            if collisions:
                print(f"EMERGENCY: Immediate collision detected with {len(collisions)} cars!")
            
            if predicted_collisions:
                closest_collision = min(predicted_collisions, key=lambda x: x[2])
                _, _, time = closest_collision
                print(f"WARNING: Collision predicted in {time} steps - immediate action needed")
            
            # Update the smart car
            self.smart_car.update(vehicle_list)
            
            # Update our target tracking to match the smart car
            self.current_target = self.smart_car.current_target
            
            # Check if route has been modified by collision avoidance
            if self.smart_car.route != self.route:
                self.route = self.smart_car.route
            
        except Exception as e:
            print(f"Error updating SmartCar: {e}")
            import traceback
            traceback.print_exc()
    
    def draw_scene(self):
        """Draw the complete simulation scene."""
        # Clear screen
        self.renderer.clear_screen()
        
        # Draw roads
        for road in self.roads:
            start, end = road
            self.renderer.draw_road(start, end, self.ROAD_WIDTH)
        
        # Draw intersections
        for intersection in self.intersections:
            self.renderer.draw_intersection(intersection)
        
        # Draw white car targets
        for car in self.white_cars:
            if len(car.route) > car.current_target:
                target = car.route[car.current_target]
                self.renderer.draw_waypoint(target, (255, 165, 0))
        
        # Draw start and end points
        if self.start_point:
            self.renderer.draw_waypoint(self.start_point, self.renderer.RED, 20)
        if self.end_point:
            self.renderer.draw_waypoint(self.end_point, self.renderer.BLUE, 20)
        
        # Draw route for main car
        if self.route:
            self.renderer.draw_route(self.route, self.renderer.PURPLE)
            
            # Highlight current target
            if self.current_target < len(self.route):
                self.renderer.draw_waypoint(self.route[self.current_target], self.renderer.RED, 8)
                
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
            # Draw ghost danger zone (only when hitboxes are shown)
            if hasattr(self, 'show_hitboxes') and self.show_hitboxes:
                danger_radius = self.ROAD_WIDTH * 5
                self.renderer.draw_ghost_danger_zone(car.x, car.y, danger_radius)
            
            # Draw the car
            self.draw_car(car.x, car.y, car.angle, self.renderer.WHITE)
            
            # Draw hitbox for debugging
            if hasattr(self, 'show_hitboxes') and self.show_hitboxes:
                corners = self.collision_detector.get_car_corners(car.x, car.y, car.angle)
                self.renderer.draw_car_corners(corners, (255, 165, 0))
                
                # Draw safety zone around white cars
                safety_corners = self.collision_detector.get_car_corners(
                    car.x, car.y, car.angle, safety_margin=self.safety_zone_size)
                self.renderer.draw_car_corners(safety_corners, (255, 100, 0), 1)
        
        # Draw main car (purple)
        if self.smart_car is not None:
            collision_color = self.renderer.PURPLE
            
            # Change car color based on collision status
            if hasattr(self.smart_car, 'collision_risk') and self.smart_car.collision_risk:
                collision_color = (255, 50, 255)  # Brighter purple
                
                # Draw collision avoidance indicator
                if hasattr(self.smart_car, 'current_strategy'):
                    if self.smart_car.current_strategy == self.CollisionStrategy.SLOW_DOWN:
                        self.renderer.draw_car_indicator(self.smart_car.x, self.smart_car.y, 'SLOW')
                    elif self.smart_car.current_strategy == self.CollisionStrategy.YIELD:
                        self.renderer.draw_car_indicator(self.smart_car.x, self.smart_car.y, 'STOP')
                    elif self.smart_car.current_strategy == self.CollisionStrategy.REROUTE:
                        self.renderer.draw_car_indicator(self.smart_car.x, self.smart_car.y, 'REROUTE')
            
            # Check if we're slowing down for a turn
            if self.current_target < len(self.route) - 1 and self.smart_car.speed < self.smart_car.default_speed * 0.9:
                # If no other indicator is showing
                if not hasattr(self.smart_car, 'collision_risk') or not self.smart_car.collision_risk:
                    self.renderer.draw_car_indicator(self.smart_car.x, self.smart_car.y, 'TURN')
            
            # Draw the car
            self.draw_car(self.smart_car.x, self.smart_car.y, self.smart_car.angle, collision_color)
            
            # Draw hitbox for debugging
            if hasattr(self, 'show_hitboxes') and self.show_hitboxes:
                corners = self.collision_detector.get_car_corners(
                    self.smart_car.x, self.smart_car.y, self.smart_car.angle)
                # Use red hitbox when collision risk is detected
                hitbox_color = (255, 0, 0) if hasattr(self.smart_car, 'collision_risk') and self.smart_car.collision_risk else (255, 165, 0)
                self.renderer.draw_car_corners(corners, hitbox_color)
                
                # Draw detection zone
                detection_corners = self.collision_detector.get_car_corners(
                    self.smart_car.x, self.smart_car.y, self.smart_car.angle, 
                    safety_margin=self.detection_zone_size)
                self.renderer.draw_car_corners(detection_corners, (100, 100, 255), 1)
                
                # Add expanded blue detection circle
                self.renderer.draw_detection_zone(
                    self.smart_car.x, self.smart_car.y, self.detection_range)
        
        # Draw UI elements
        grid_dropdown_rect = self.ui_manager.get_grid_dropdown_rect()
        self.renderer.draw_dropdown(
            grid_dropdown_rect, 
            self.ui_manager.get_grid_dropdown_text(),
            self.ui_manager.is_grid_dropdown_open(),
            self.ui_manager.grid_sizes)
        
        car_dropdown_rect = self.ui_manager.get_car_dropdown_rect()
        self.renderer.draw_dropdown(
            car_dropdown_rect, 
            self.ui_manager.get_car_dropdown_text(),
            self.ui_manager.is_car_dropdown_open(),
            self.ui_manager.car_counts)
        
        # Draw instructions
        self.renderer.draw_instructions(self.ui_manager.get_instructions())
            
        # Display collision avoidance status if active
        if self.smart_car and hasattr(self.smart_car, 'collision_risk') and self.smart_car.collision_risk:
            if hasattr(self.smart_car, 'current_strategy'):
                status_text = f"Collision Avoidance Active: "
                
                if self.smart_car.current_strategy == self.CollisionStrategy.SLOW_DOWN:
                    status_text += "SLOWING DOWN"
                    status_color = (255, 165, 0)  # Orange
                elif self.smart_car.current_strategy == self.CollisionStrategy.YIELD:
                    status_text += "YIELDING"
                    status_color = (255, 0, 0)  # Red 
                elif self.smart_car.current_strategy == self.CollisionStrategy.REROUTE:
                    status_text += "REROUTING"
                    status_color = (0, 100, 255)  # Blue
                
                status_surface = self.renderer.medium_font.render(status_text, True, status_color)
                self.screen.blit(status_surface, (self.width - status_surface.get_width() - 20, 20))
                
        # Display route metrics
        if self.route and self.smart_car:
            # Calculate safety and efficiency metrics
            obstacle_positions = [(car.x, car.y) for car in self.white_cars]
            safety_score = self.world_model.calculate_path_safety(self.route, obstacle_positions)
            efficiency_score = self.world_model.calculate_path_efficiency(self.route)
            combined_score = (safety_score * 0.6) + (efficiency_score * 0.4)
            
            # Display the metrics
            self.renderer.draw_metrics(safety_score, efficiency_score, combined_score)
    
    def reset_simulation(self):
        """Reset the simulation to initial conditions."""
        # Reset route data
        self.start_point = None
        self.end_point = None
        self.route = []
        self.current_target = 0
        self.smart_car = None
        
        # Update grid size and regenerate the grid
        self.world_model.update_grid_size(self.ui_manager.get_grid_size())
        self.intersections, self.roads = self.world_model.intersections, self.world_model.roads
        
        # Create new white cars
        self.white_cars = self.create_white_cars()
    
    def update_car_count(self):
        """Update the number of cars in the simulation."""
        self.white_cars = self.create_white_cars(self.ui_manager.get_car_count())
    
    def handle_click(self, pos):
        """
        Handle a mouse click event
        
        Args:
            pos (tuple): Mouse position (x, y)
        """
        # Use UI manager to handle click on UI elements
        grid_changed, car_count_changed, need_update = self.ui_manager.handle_click(
            pos, self.intersections, self.handle_intersection_click)
            
        if grid_changed:
            # Update the grid size
            self.reset_simulation()
        elif car_count_changed:
            # Update the car count
            self.update_car_count()
    
    def handle_intersection_click(self, intersection):
        """
        Handle a click on an intersection
        
        Args:
            intersection (tuple): The intersection that was clicked
        """
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
    
    def get_car_corners(self, x, y, angle, safety_margin=1.0):
        """
        Calculate the four corners of a car for collision detection.
        Wrapper for collision_detector.get_car_corners for backwards compatibility.
        
        Args:
            x (float): X coordinate
            y (float): Y coordinate
            angle (float): Angle in degrees
            safety_margin (float): Multiplier for car size
            
        Returns:
            list: Four corner points of the car
        """
        return self.collision_detector.get_car_corners(x, y, angle, safety_margin)
    
    def check_collision(self, car1, car2):
        """
        Check if two cars are colliding.
        Wrapper for collision_detector.check_collision for backwards compatibility.
        
        Args:
            car1: First vehicle
            car2: Second vehicle
            
        Returns:
            bool: True if collision detected, False otherwise
        """
        return self.collision_detector.check_collision(car1, car2)
    
    def predict_collision(self, car1, car2, time_steps=15):
        """
        Predict if two cars will collide within a few time steps.
        Wrapper for collision_detector.predict_collision for backwards compatibility.
        
        Args:
            car1: First vehicle
            car2: Second vehicle
            time_steps (int): Number of time steps to look ahead
            
        Returns:
            tuple: (will_collide, time_to_collision)
        """
        return self.collision_detector.predict_collision(car1, car2, time_steps)
    
    def check_all_collisions(self):
        """
        Check for collisions between the main car and all white cars.
        
        Returns:
            tuple: (immediate_collisions, predicted_collisions)
        """
        # Use our collision detector with a compatibility wrapper
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
                    self.smart_car.current_strategy = self.CollisionStrategy.YIELD
                
                # Check for predicted collisions
                else:
                    # Check if car is within detection range
                    dx = car.x - self.smart_car.x
                    dy = car.y - self.smart_car.y
                    distance = math.sqrt(dx*dx + dy*dy)
                    
                    # Only consider cars within the detection range
                    if distance <= self.detection_range:
                        will_collide, time_to_collision = self.predict_collision(self.smart_car, car)
                        if will_collide:
                            predicted_collisions.append((self.smart_car, car, time_to_collision))
                            
                            # Set appropriate strategy based on time to collision
                            self.smart_car.collision_risk = True
                            
                            # Very close collision - slow down significantly
                            if time_to_collision <= 7:
                                self.smart_car.current_strategy = self.CollisionStrategy.SLOW_DOWN
                                
                            # Further collision - consider rerouting
                            elif time_to_collision <= 12:
                                # Find the nearest intersection
                                current_pos = (self.smart_car.x, self.smart_car.y)
                                nearest_intersection = self.world_model.find_nearest_intersection(current_pos)
                                
                                if nearest_intersection:
                                    # Check distance to nearest intersection
                                    dx = current_pos[0] - nearest_intersection[0]
                                    dy = current_pos[1] - nearest_intersection[1]
                                    dist_to_intersection = math.sqrt(dx*dx + dy*dy)
                                    
                                    # Only try to reroute if we're close to an intersection
                                    if dist_to_intersection < self.ROAD_WIDTH * 0.8:
                                        # We're close to an intersection, rerouting is reasonable
                                        if self.smart_car.current_strategy != self.CollisionStrategy.REROUTE:
                                            # Trigger rerouting logic
                                            self.initiate_reroute(self.smart_car, car)
                                    else:
                                        # We're in the middle of a road, just slow down
                                        self.smart_car.current_strategy = self.CollisionStrategy.SLOW_DOWN
                                else:
                                    # No intersection found, just slow down
                                    self.smart_car.current_strategy = self.CollisionStrategy.SLOW_DOWN
        
        return collisions, predicted_collisions
    
    def initiate_reroute(self, smart_car, obstacle_car):
        """
        Initiate rerouting to avoid a potential collision
        
        Args:
            smart_car: The main car
            obstacle_car: The car to avoid
            
        Returns:
            bool: True if rerouted, False otherwise
        """
        # This method would contain the implementation of rerouting
        # Here, we just set the strategy
        smart_car.current_strategy = self.CollisionStrategy.REROUTE
        return False
    
    def preventative_reroute(self):
        """
        Proactively reroute to avoid white cars in the planned route
        
        Returns:
            bool: True if rerouted, False otherwise
        """
        # This method would contain preventative rerouting logic
        # For compatibility, we keep it as a stub
        return False
    
    def random_start_finish(self):
        """Randomly select start and finish points with a route between them."""
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
            
            # Find path
            path = self.find_path(self.start_point, self.end_point)
            if path:
                self.route = path
                self.current_target = 1  # Skip start point
                # Update the smart car's route
                self.smart_car.route = path
                self.smart_car.original_route = path.copy()
                self.smart_car.current_target = 1
                print(f"Found route with {len(path)} points")
                return True
            else:
                print("No valid path found between random points")
                return False
                
        except Exception as e:
            print(f"Error creating SmartCar with random route: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def run(self):
        """Run the main simulation loop."""
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
                            print(f"Hitboxes {'on' if self.show_hitboxes else 'off'}")
                        elif event.key == pygame.K_r:
                            # Force purple car to reroute (for testing)
                            print("Forcing reroute!")
                        elif event.key == pygame.K_p:
                            # Force preventative reroute check
                            print("Checking route safety and rerouting if needed...")
                            self.preventative_reroute()
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
            
    def validate_routes(self):
        """Validate all vehicle routes to ensure they follow roads."""
        for car in self.white_cars:
            if len(car.route) < 2:
                continue
                
            # Check each segment
            for i in range(len(car.route) - 1):
                if not self.has_road_between(car.route[i], car.route[i+1]):
                    # Invalid segment found - print and correct
                    print(f"Invalid route segment: {car.route[i]} to {car.route[i+1]}")
                    
                    # Fix by generating a new route
                    new_route = self.world_model.generate_random_route()
                    car.route = new_route
                    car.vehicle.route = new_route
                    car.vehicle.current_target = 1
                    car.current_target = 1
                    break


def main(args=None):
    """Main function to initialize and run the node."""
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