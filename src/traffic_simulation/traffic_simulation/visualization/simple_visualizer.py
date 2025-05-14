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
        
        # Generate the map
        self.intersections, self.roads = self.generate_grid()
        
        # Create white cars
        self.white_cars = self.create_white_cars()  # Create 8 white cars
        
        # For FPS control
        self.clock = pygame.time.Clock()
    
    def generate_grid(self):
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
        
        # Connect adjacent intersections
        for i in range(len(intersections)):
            for j in range(i+1, len(intersections)):
                x1, y1 = intersections[i]
                x2, y2 = intersections[j]
                
                # Only connect if they're adjacent
                dx = abs(x1 - x2)
                dy = abs(y1 - y2)
                
                if (dx <= cell_width * 1.5 and dy <= cell_height * 1.5) and random.random() < 0.7:
                    roads.append((intersections[i], intersections[j]))
        
        return intersections, roads
    
    def find_path(self, start, end):
        # Simple breadth-first search
        visited = set()
        queue = [(start, [start])]
        
        while queue:
            current, path = queue.pop(0)
            
            if current == end:
                return path
                
            if current in visited:
                continue
                
            visited.add(current)
            
            # Find neighbors
            neighbors = []
            for road in self.roads:
                a, b = road
                if a == current and b not in visited:
                    neighbors.append(b)
                elif b == current and a not in visited:
                    neighbors.append(a)
                    
            # Add neighbors to queue
            for neighbor in neighbors:
                queue.append((neighbor, path + [neighbor]))
        
        return None  # No path found
    
    def create_white_cars(self, count=None):
        # Use the current_car_count if count is not specified
        if count is None:
            count = self.current_car_count
            
        cars = []
        for _ in range(count):
            route = self.generate_random_route()
            # Use our WhiteCar class from the vehicles module
            car = WhiteCar(route[0][0], route[0][1], route, self)
            cars.append(car)
        return cars
    
    def generate_random_route(self):
        """Generate a valid route along roads for vehicles to follow"""
        # Try a few times to find valid start/end points with a path
        for _ in range(10):  # Try up to 10 times
            start = random.choice(self.intersections)
            end = random.choice(self.intersections)
            while start == end:  # Make sure start and end are different
                end = random.choice(self.intersections)
            
            path = self.find_path(start, end)
            if path and len(path) > 1:  # Valid path found
                return path
        
        # If we couldn't find a good path after multiple attempts, 
        # find any connected points to ensure we stay on roads
        for start in self.intersections:
            for road in self.roads:
                if start in road:
                    # Find the other end of this road
                    other_end = road[1] if road[0] == start else road[0]
                    return [start, other_end]
        
        # Fallback to a single point if nothing else works
        # This will make the car effectively stay in place
        start = random.choice(self.intersections)
        return [start, start]
    
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
            
            # Get real vehicle objects for collision detection
            vehicle_list = [car.vehicle for car in self.white_cars]
            
            # Update the smart car with collision avoidance
            self.smart_car.update(vehicle_list)
            
            # Update our target tracking to match the smart car
            self.current_target = self.smart_car.current_target
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
        
        # Draw white cars
        for car in self.white_cars:
            self.draw_car(car.x, car.y, car.angle, self.WHITE)
        
        # Draw main car (purple)
        if self.smart_car is not None:
            self.draw_car(self.smart_car.x, self.smart_car.y, self.smart_car.angle, self.PURPLE)
        
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
            "Click dropdowns to change grid size and car count",
            "ESC to exit"
        ]
        
        for i, text in enumerate(instructions):
            text_surface = font.render(text, True, self.WHITE)
            self.screen.blit(text_surface, (20, 20 + i * 20))
    
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
        
        try:
            while running:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            running = False
                    elif event.type == pygame.MOUSEBUTTONDOWN:
                        self.handle_click(pygame.mouse.get_pos())
                
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
        except Exception as e:
            self.get_logger().error(f"Error in visualization: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())


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
                # Generate a new route for continuous movement
                self.route = self.node.generate_random_route()
                self.vehicle.route = self.route
                self.vehicle.current_target = 1  # Skip the start point
                self.current_target = 1
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