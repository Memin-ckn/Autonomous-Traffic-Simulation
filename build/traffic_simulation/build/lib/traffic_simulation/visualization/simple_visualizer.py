#!/usr/bin/env python3

import pygame
import sys
import math
import random
import rclpy
from rclpy.node import Node

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
        self.car_counts = [4, 8, 12, 16, 20, 24]
        self.current_car_count = 8  # Default car count
        self.car_dropdown_open = False
        
        # Dropdown dimensions
        self.dropdown_width = int(self.width / 6)
        self.dropdown_height = int(self.height / 20)
        
        # Car properties
        self.CAR_WIDTH = 60
        self.CAR_HEIGHT = 30
        
        # Main car
        self.main_car_x = None
        self.main_car_y = None
        self.main_car_angle = 0
        self.main_car_speed = 3
        
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
            car = WhiteCar(route[0][0], route[0][1], route)
            cars.append(car)
        return cars
    
    def generate_random_route(self):
        start = random.choice(self.intersections)
        end = random.choice(self.intersections)
        while start == end:  # Make sure start and end are different
            end = random.choice(self.intersections)
        
        path = self.find_path(start, end)
        if not path:
            return [start, end]  # Fallback to direct path
        return path
    
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
        if self.main_car_x is None or self.main_car_y is None or self.current_target >= len(self.route):
            return
        
        # Get current target
        target_x, target_y = self.route[self.current_target]
        
        # Calculate direction to target
        dx = target_x - self.main_car_x
        dy = target_y - self.main_car_y
        target_angle = math.degrees(math.atan2(dy, dx))
        
        # Rotate car towards target (smooth turning)
        angle_diff = (target_angle - self.main_car_angle + 180) % 360 - 180
        if abs(angle_diff) > 5:
            self.main_car_angle += angle_diff * 0.1
        else:
            self.main_car_angle = target_angle
        
        # Move car forward
        self.main_car_x += math.cos(math.radians(self.main_car_angle)) * self.main_car_speed
        self.main_car_y += math.sin(math.radians(self.main_car_angle)) * self.main_car_speed
        
        # Check if reached target
        distance = math.sqrt(dx*dx + dy*dy)
        if distance < 20:
            self.current_target += 1
    
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
        if self.main_car_x is not None and self.main_car_y is not None:
            self.draw_car(self.main_car_x, self.main_car_y, self.main_car_angle, self.PURPLE)
        
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
        """Reset the simulation with the current grid size"""
        # Clear existing data
        self.start_point = None
        self.end_point = None
        self.route = []
        self.main_car_x = None
        self.main_car_y = None
        self.current_target = 0
        
        # Regenerate grid and cars
        self.intersections, self.roads = self.generate_grid()
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
        
        # Check for car count dropdown menu clicks
        car_dropdown_rect = pygame.Rect(
            int(self.width * 0.05) + self.dropdown_width + 20,
            int(self.height * 0.05),
            self.dropdown_width,
            self.dropdown_height
        )
        
        # Handle grid size dropdown toggle click
        if grid_dropdown_rect.collidepoint(pos):
            self.dropdown_open = not self.dropdown_open
            # Close the other dropdown if open
            self.car_dropdown_open = False
            return
        
        # Handle car count dropdown toggle click
        if car_dropdown_rect.collidepoint(pos):
            self.car_dropdown_open = not self.car_dropdown_open
            # Close the other dropdown if open
            self.dropdown_open = False
            return
        
        # Handle grid size dropdown option clicks
        if self.dropdown_open:
            for i, size in enumerate(self.grid_sizes):
                option_rect = pygame.Rect(
                    grid_dropdown_rect.x,
                    grid_dropdown_rect.y + (i + 1) * self.dropdown_height,
                    self.dropdown_width,
                    self.dropdown_height
                )
                if option_rect.collidepoint(pos):
                    self.current_grid_size = size
                    self.dropdown_open = False
                    self.reset_simulation()
                    return
            
            # Click outside dropdown options, close dropdown
            self.dropdown_open = False
            return
        
        # Handle car count dropdown option clicks
        if self.car_dropdown_open:
            for i, count in enumerate(self.car_counts):
                option_rect = pygame.Rect(
                    car_dropdown_rect.x,
                    car_dropdown_rect.y + (i + 1) * self.dropdown_height,
                    self.dropdown_width,
                    self.dropdown_height
                )
                if option_rect.collidepoint(pos):
                    self.current_car_count = count
                    self.car_dropdown_open = False
                    self.update_car_count()
                    return
            
            # Click outside dropdown options, close dropdown
            self.car_dropdown_open = False
            return
        
        # Check if clicked near an intersection
        for intersection in self.intersections:
            dx = pos[0] - intersection[0]
            dy = pos[1] - intersection[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < 25:
                if not self.start_point:
                    # Set start point
                    self.start_point = intersection
                    self.main_car_x, self.main_car_y = intersection
                    self.main_car_angle = 0
                    self.current_target = 0
                    self.route = []
                elif not self.end_point:
                    # Set end point
                    self.end_point = intersection
                    
                    # Find path
                    path = self.find_path(self.start_point, self.end_point)
                    if path:
                        self.route = path
                        self.current_target = 1  # Skip the start point
                    else:
                        # No path found, just go directly
                        self.route = [self.start_point, self.end_point]
                        self.current_target = 1
                else:
                    # Reset
                    self.start_point = intersection
                    self.end_point = None
                    self.main_car_x, self.main_car_y = intersection
                    self.main_car_angle = 0
                    self.current_target = 0
                    self.route = []
                break
    
    def run(self):
        running = True
        
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


class WhiteCar:
    def __init__(self, x, y, route):
        self.x = x
        self.y = y
        self.angle = 0
        self.speed = random.uniform(1.0, 2.5)
        self.route = route
        self.current_target = 1  # Skip start point
        
    def update(self):
        # Check if reached end of route
        if self.current_target >= len(self.route):
            # Reset to beginning
            self.current_target = 0
            return
        
        # Get current target
        target_x, target_y = self.route[self.current_target]
        
        # Calculate direction to target
        dx = target_x - self.x
        dy = target_y - self.y
        target_angle = math.degrees(math.atan2(dy, dx))
        
        # Rotate car towards target (smooth turning)
        angle_diff = (target_angle - self.angle + 180) % 360 - 180
        if abs(angle_diff) > 5:
            self.angle += angle_diff * 0.1
        else:
            self.angle = target_angle
        
        # Move car forward
        self.x += math.cos(math.radians(self.angle)) * self.speed
        self.y += math.sin(math.radians(self.angle)) * self.speed
        
        # Check if reached target
        distance = math.sqrt(dx*dx + dy*dy)
        if distance < 20:
            self.current_target += 1


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SimpleVisualizer()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
        pygame.quit()


if __name__ == '__main__':
    main() 