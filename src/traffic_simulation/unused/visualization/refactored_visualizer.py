#!/usr/bin/env python3

"""
Refactored Traffic Simulation Visualizer.
Main ROS2 node that implements the traffic simulation visualization using pygame.
"""

import pygame
import sys
import rclpy
from rclpy.node import Node
from enum import Enum

# Import our components
from traffic_simulation.visualization.components.renderer import SimulationRenderer
from traffic_simulation.visualization.components.world_model import WorldModel
from traffic_simulation.visualization.components.collision_detector import CollisionDetector
from traffic_simulation.visualization.components.ui_manager import UIManager

# Import vehicle classes
try:
    from traffic_simulation.vehicles.vehicle import Vehicle, WhiteCar, SmartCar
    from traffic_simulation.sensors.sensor import Lidar, Radar
    from traffic_simulation.communication.iot import IoTCommunicator
    from traffic_simulation.planning.collision_avoidance import CollisionAvoidance, CollisionStrategy
    IMPORT_SUCCESS = True
except Exception as e:
    print(f"Import error: {e}")
    import traceback
    traceback.print_exc()
    IMPORT_SUCCESS = False
    
    # Define a backup CollisionStrategy enum
    class CollisionStrategy(Enum):
        SLOW_DOWN = 1
        REROUTE = 2
        YIELD = 3


class RefactoredVisualizer(Node):
    """
    Main visualization node for the traffic simulation.
    Integrates various components to create a cohesive simulation.
    """
    def __init__(self):
        """Initialize the visualization node and components."""
        super().__init__('refactored_visualizer')
        pygame.init()
        
        # Create a fullscreen window
        self.screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
        self.width, self.height = self.screen.get_size()
        pygame.display.set_caption("Traffic Simulator")
        
        # Initialize components
        self.init_components()
        
        # Initialize simulation state
        self.init_simulation_state()
        
        # For FPS control
        self.clock = pygame.time.Clock()
        
        # Debug options
        self.show_hitboxes = False  # Toggle with 'H' key
        
        # Create white cars
        self.white_cars = self.create_white_cars(self.ui_manager.get_car_count())
        
    def init_components(self):
        """Initialize all the components for the simulation."""
        # Renderer for drawing
        self.renderer = SimulationRenderer(self.screen, self.width, self.height)
        
        # World model for roads and pathfinding
        self.world_model = WorldModel(self.width, self.height, grid_size=4)
        
        # Car dimensions
        self.CAR_WIDTH = 60
        self.CAR_HEIGHT = 30
        
        # Road width
        self.ROAD_WIDTH = 40
        
        # Collision detector
        self.collision_detector = CollisionDetector(self.CAR_WIDTH, self.CAR_HEIGHT, self.ROAD_WIDTH)
        
        # UI manager
        self.ui_manager = UIManager(self.width, self.height)
        
        # Import CollisionStrategy enum if possible
        try:
            from traffic_simulation.planning.collision_avoidance import CollisionStrategy
            self.CollisionStrategy = CollisionStrategy
        except Exception as e:
            print(f"Failed to import CollisionStrategy: {e}")
            # Use the backup enum defined earlier
            self.CollisionStrategy = CollisionStrategy
    
    def init_simulation_state(self):
        """Initialize the simulation state variables."""
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
        
        # Detection and avoidance parameters
        self.detection_range = self.ROAD_WIDTH * 12
        self.safety_zone_size = 2.0
        self.detection_zone_size = 4.0
        
        # Movement smoothing parameters
        self.acceleration_rate = 0.05
        self.deceleration_rate = 0.1
        self.min_speed = 0.1
        
        # Route planning parameters
        self.route_check_interval = 30
        self.preventative_reroute_enabled = True
        self.route_safety_threshold = 50
        self.route_check_count = 0
    
    def create_white_cars(self, count=None):
        """
        Create white cars with valid routes
        
        Args:
            count (int): Number of cars to create
            
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
                if not self.world_model.has_road_between(route[i], route[i+1]):
                    print(f"Invalid route generated: {route[i]} to {route[i+1]}")
                    valid_route = False
                    break
            
            # If invalid, try once more with a simple route
            if not valid_route and self.world_model.roads:
                road = self.world_model.roads[0]  # Take the first road
                route = list(road)    # Convert tuple to list
            
            # Use our WhiteCar class from the vehicles module
            if IMPORT_SUCCESS:
                car = WhiteCar(route[0][0], route[0][1], route, self)
                
                # Pass map data to the car
                car.set_map_data(self.world_model.intersections, self.world_model.roads)
            else:
                # Create a simple fallback car if imports failed
                car = self.create_fallback_car(route[0][0], route[0][1], route)
            
            cars.append(car)
        return cars
    
    def create_fallback_car(self, x, y, route):
        """
        Create a fallback car when imports fail
        
        Args:
            x (float): X coordinate
            y (float): Y coordinate
            route (list): Route for the car
            
        Returns:
            object: Simple car object
        """
        class SimpleCar:
            def __init__(self, x, y, route):
                self.x = x
                self.y = y
                self.angle = 0
                self.route = route
                self.current_target = 1
                self.vx = 0
                self.vy = 0
                self.speed = 2.0
                self.vehicle = self  # Reference to self for compatibility
                
            def update(self, other_vehicles=None):
                if self.current_target < len(self.route):
                    target = self.route[self.current_target]
                    dx = target[0] - self.x
                    dy = target[1] - self.y
                    distance = (dx**2 + dy**2)**0.5
                    
                    if distance < 10:  # If close to target
                        self.current_target += 1
                    else:
                        # Move towards target
                        angle = math.atan2(dy, dx)
                        self.angle = math.degrees(angle)
                        self.vx = math.cos(angle) * self.speed
                        self.vy = math.sin(angle) * self.speed
                        self.x += self.vx
                        self.y += self.vy
                        
            def set_map_data(self, intersections, roads):
                # Dummy method for compatibility
                pass
                
        return SimpleCar(x, y, route)
    
    def update_main_car(self):
        """Update the main car (purple car) position and handle collision avoidance."""
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
                    # If we've already rerouted preventatively, skip the rest to avoid double-handling
                    self.current_target = self.smart_car.current_target
                    return
            
            # Check for collisions with white cars
            collisions, predicted_collisions = self.collision_detector.check_all_collisions(
                self.smart_car, self.white_cars, self.CollisionStrategy)
            
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
        for road in self.world_model.roads:
            start, end = road
            self.renderer.draw_road(start, end, self.ROAD_WIDTH)
        
        # Draw intersections
        for intersection in self.world_model.intersections:
            self.renderer.draw_intersection(intersection)
        
        # Draw each white car's current target
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
        
        # Draw white cars
        for car in self.white_cars:
            # Draw ghost danger zone if hitboxes are shown
            if self.show_hitboxes:
                danger_radius = self.ROAD_WIDTH * 5
                self.renderer.draw_ghost_danger_zone(car.x, car.y, danger_radius)
            
            # Draw the car
            self.renderer.draw_car(car.x, car.y, car.angle, self.renderer.WHITE, 
                                  self.CAR_WIDTH, self.CAR_HEIGHT)
            
            # Draw hitbox for debugging
            if self.show_hitboxes:
                corners = self.collision_detector.get_car_corners(car.x, car.y, car.angle)
                self.renderer.draw_car_corners(corners, (255, 165, 0))
                
                # Draw safety zone
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
            
            # Draw main car
            self.renderer.draw_car(self.smart_car.x, self.smart_car.y, self.smart_car.angle, 
                                  collision_color, self.CAR_WIDTH, self.CAR_HEIGHT)
            
            # Draw hitbox for debugging
            if self.show_hitboxes:
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
                
                self.renderer.draw_status_text(status_text, status_color, 
                                             (self.width - 300, 20))
                
        # Display route metrics if available
        if self.route and self.smart_car:
            # Calculate safety and efficiency metrics
            if IMPORT_SUCCESS:
                # Get white car positions for safety calculation
                obstacle_positions = [(car.x, car.y) for car in self.white_cars]
                
                # Calculate safety score
                safety_score = self.world_model.calculate_path_safety(self.route, obstacle_positions)
                
                # Calculate efficiency score
                efficiency_score = self.world_model.calculate_path_efficiency(self.route)
                
                # Calculate combined score
                combined_score = (safety_score * 0.6) + (efficiency_score * 0.4)
                
                # Draw the metrics
                self.renderer.draw_metrics(safety_score, efficiency_score, combined_score)
    
    def reset_simulation(self):
        """Reset the simulation state to initial conditions."""
        # Reset route data
        self.start_point = None
        self.end_point = None
        self.route = []
        self.current_target = 0
        self.smart_car = None
        
        # Regenerate the grid
        self.world_model.update_grid_size(self.ui_manager.get_grid_size())
        
        # Create new white cars
        self.white_cars = self.create_white_cars()
    
    def update_car_count(self):
        """Update the number of cars in the simulation."""
        self.white_cars = self.create_white_cars()
    
    def handle_intersection_click(self, intersection):
        """
        Handle a click on an intersection
        
        Args:
            intersection (tuple): The intersection point (x, y) that was clicked
        """
        # If no start point, set as start
        if self.start_point is None:
            # Set start point
            self.start_point = intersection
            try:
                # Create the smart car at the start point
                print(f"Creating SmartCar at {intersection[0]}, {intersection[1]}")
                if IMPORT_SUCCESS:
                    self.smart_car = SmartCar(intersection[0], intersection[1], [], self)
                    self.smart_car.set_map_data(self.world_model.intersections, self.world_model.roads)
                else:
                    self.smart_car = self.create_fallback_smart_car(intersection[0], intersection[1])
                print("SmartCar created successfully")
            except Exception as e:
                print(f"Error creating SmartCar: {e}")
                import traceback
                traceback.print_exc()
                # Create a backup simple car
                self.smart_car = self.create_fallback_smart_car(intersection[0], intersection[1])
            self.current_target = 0
            self.route = []
            return
            
        # If start point exists but no end point
        elif self.end_point is None and intersection != self.start_point:
            # Set end point
            self.end_point = intersection
            
            # Find path
            path = self.world_model.find_path(self.start_point, self.end_point)
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
                    if IMPORT_SUCCESS:
                        self.smart_car = SmartCar(intersection[0], intersection[1], [], self)
                        self.smart_car.set_map_data(self.world_model.intersections, self.world_model.roads)
                    else:
                        self.smart_car = self.create_fallback_smart_car(intersection[0], intersection[1])
                except Exception as e:
                    print(f"Error creating SmartCar: {e}")
                    import traceback
                    traceback.print_exc()
                    # Create a backup simple car
                    self.smart_car = self.create_fallback_smart_car(intersection[0], intersection[1])
                self.current_target = 0
                self.route = []
            return
            
        # If both start and end exist, reset and set as new start
        else:
            self.start_point = intersection
            self.end_point = None
            try:
                # Create a new smart car at this location
                if IMPORT_SUCCESS:
                    self.smart_car = SmartCar(intersection[0], intersection[1], [], self)
                    self.smart_car.set_map_data(self.world_model.intersections, self.world_model.roads)
                else:
                    self.smart_car = self.create_fallback_smart_car(intersection[0], intersection[1])
            except Exception as e:
                print(f"Error creating SmartCar: {e}")
                import traceback
                traceback.print_exc()
                # Create a backup simple car
                self.smart_car = self.create_fallback_smart_car(intersection[0], intersection[1])
            self.current_target = 0
            self.route = []
            return
    
    def create_fallback_smart_car(self, x, y):
        """
        Create a fallback SmartCar when imports fail
        
        Args:
            x (float): X coordinate
            y (float): Y coordinate
            
        Returns:
            object: Simple SmartCar object
        """
        class SimpleSmartCar:
            def __init__(self, x, y):
                self.x = x
                self.y = y
                self.angle = 0
                self.route = []
                self.original_route = []
                self.current_target = 0
                self.vx = 0
                self.vy = 0
                self.speed = 3.0
                self.default_speed = 3.0
                self.collision_risk = False
                self.current_strategy = None
                self.id = "purple_car"
                
            def update(self, other_vehicles=None):
                if self.current_target < len(self.route):
                    target = self.route[self.current_target]
                    dx = target[0] - self.x
                    dy = target[1] - self.y
                    distance = (dx**2 + dy**2)**0.5
                    
                    if distance < 10:  # If close to target
                        self.current_target += 1
                    else:
                        # Move towards target
                        angle = math.atan2(dy, dx)
                        self.angle = math.degrees(angle)
                        self.vx = math.cos(angle) * self.speed
                        self.vy = math.sin(angle) * self.speed
                        self.x += self.vx
                        self.y += self.vy
                        
            def set_map_data(self, intersections, roads):
                # Dummy method for compatibility
                pass
                
        return SimpleSmartCar(x, y)
    
    def preventative_reroute(self):
        """
        Proactively reroute to avoid white cars in the planned route
        
        Returns:
            bool: True if rerouted, False otherwise
        """
        # This is a stub for the preventative rerouting functionality
        # In a full implementation, this would check for potential conflicts
        # and reroute the smart car if necessary
        return False
    
    def random_start_finish(self):
        """Randomly select start and finish points and generate a route."""
        if not self.world_model.intersections or len(self.world_model.intersections) < 2:
            print("Not enough intersections to create a random route")
            return False
            
        # Clear existing route
        self.start_point = None
        self.end_point = None
        self.route = []
        self.current_target = 0
        
        # Select random start point
        self.start_point = random.choice(self.world_model.intersections)
        
        # Select random end point (different from start)
        available_ends = [p for p in self.world_model.intersections if p != self.start_point]
        if not available_ends:
            print("No valid end points available")
            return False
            
        self.end_point = random.choice(available_ends)
        
        print(f"Selected random start: {self.start_point}, end: {self.end_point}")
        
        # Create the smart car at the start point
        try:
            if IMPORT_SUCCESS:
                self.smart_car = SmartCar(self.start_point[0], self.start_point[1], [], self)
                self.smart_car.set_map_data(self.world_model.intersections, self.world_model.roads)
            else:
                self.smart_car = self.create_fallback_smart_car(self.start_point[0], self.start_point[1])
            
            # Find path
            path = self.world_model.find_path(self.start_point, self.end_point)
            if path:
                self.route = path
                self.current_target = 1  # Skip start point
                # Update the smart car's route
                self.smart_car.route = path
                self.smart_car.original_route = path.copy()
                self.smart_car.current_target = 1
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
                            # Force reroute (stub for now)
                            print("Manual reroute triggered")
                        elif event.key == pygame.K_p:
                            # Force preventative reroute check
                            print("Checking route safety and rerouting if needed...")
                            self.preventative_reroute()
                        elif event.key == pygame.K_s:
                            # Random start and finish
                            print("Generating random start and finish points...")
                            self.random_start_finish()
                    elif event.type == pygame.MOUSEBUTTONDOWN:
                        grid_changed, car_count_changed, _ = self.ui_manager.handle_click(
                            pygame.mouse.get_pos(), 
                            self.world_model.intersections,
                            self.handle_intersection_click)
                        
                        if grid_changed:
                            # Update the grid size
                            self.reset_simulation()
                        elif car_count_changed:
                            # Update the car count
                            self.update_car_count()
                
                # Update main car position
                self.update_main_car()
                
                # Update white cars
                for car in self.white_cars:
                    car.update()
                
                # Draw everything
                self.draw_scene()
                
                # Update the display
                pygame.display.flip()
                self.clock.tick(60)
        except KeyboardInterrupt:
            print("Keyboard interrupt received, shutting down")
        except Exception as e:
            self.get_logger().error(f"Error in visualization: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
        finally:
            pygame.quit()


def main(args=None):
    """Main function to initialize and run the node."""
    try:
        print("Initializing ROS2...")
        rclpy.init(args=args)
        
        print("Creating RefactoredVisualizer node...")
        node = RefactoredVisualizer()
        
        print("Starting RefactoredVisualizer run loop...")
        node.run()
        
        print("RefactoredVisualizer run completed normally")
    
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