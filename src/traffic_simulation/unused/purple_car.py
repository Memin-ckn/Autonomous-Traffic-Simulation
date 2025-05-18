#!/usr/bin/env python3

import pygame
import sys
import math
import random

# Initialize pygame
pygame.init()

# Create a fullscreen window
screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
WIDTH, HEIGHT = screen.get_size()
pygame.display.set_caption("Traffic Simulator")

# Define colors
GREEN = (34, 139, 34)  # Background
BLACK = (0, 0, 0)      # Road, outlines
WHITE = (255, 255, 255) # Road lines and cars
RED = (255, 0, 0)      # Start point
BLUE = (0, 0, 255)     # End point
PURPLE = (255, 0, 255) # Main car color
YELLOW = (255, 255, 0) # Highlights

# Car properties - SMALLER SIZE
CAR_WIDTH = 60  # Smaller width to fit in lanes
CAR_HEIGHT = 30  # Smaller height to fit in lanes

# Main car
main_car_x = None
main_car_y = None
main_car_angle = 0
main_car_speed = 3

# White cars
white_cars = []

# Route
start_point = None
end_point = None
route = []
current_target = 0

# Road width
ROAD_WIDTH = 40

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
            # If at end of route, pick a new route
            if self.current_target >= len(self.route):
                start = self.route[-1]
                end = random.choice(intersections)
                new_path = find_path(start, end, roads)
                if new_path:
                    self.route = new_path
                    self.current_target = 0

# Generate a grid of intersections
def generate_grid():
    grid_size = 4  # 4x4 grid
    intersections = []
    roads = []
    
    # Create a grid of intersection points
    cell_width = WIDTH // (grid_size + 1)
    cell_height = HEIGHT // (grid_size + 1)
    
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
            
            # Only connect if they're adjacent (horizontally, vertically or diagonally)
            dx = abs(x1 - x2)
            dy = abs(y1 - y2)
            
            if (dx <= cell_width * 1.5 and dy <= cell_height * 1.5) and random.random() < 0.7:
                roads.append((intersections[i], intersections[j]))
    
    return intersections, roads

# Create a path from start to end
def find_path(start, end, roads):
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
        for road in roads:
            a, b = road
            if a == current and b not in visited:
                neighbors.append(b)
            elif b == current and a not in visited:
                neighbors.append(a)
                
        # Add neighbors to queue
        for neighbor in neighbors:
            queue.append((neighbor, path + [neighbor]))
    
    return None  # No path found

# Draw a car
def draw_car(x, y, angle, color):
    # Convert angle from degrees to radians
    rad_angle = math.radians(angle)
    
    # Calculate car center
    center_x, center_y = x, y
    
    # Calculate corners (unrotated)
    half_width = CAR_WIDTH // 2
    half_height = CAR_HEIGHT // 2
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
    pygame.draw.polygon(screen, BLACK, rotated_corners, 0)  # Black outline
    
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
    pygame.draw.polygon(screen, color, smaller_corners, 0)
    
    # Draw front
    front_x = center_x + half_width * math.cos(rad_angle)
    front_y = center_y + half_width * math.sin(rad_angle)
    pygame.draw.circle(screen, RED, (int(front_x), int(front_y)), 8)

# Draw the whole scene
def draw_scene():
    # Clear screen
    screen.fill(GREEN)
    
    # Draw roads
    for road in roads:
        start, end = road
        pygame.draw.line(screen, BLACK, start, end, ROAD_WIDTH)  # Thick black road
        pygame.draw.line(screen, WHITE, start, end, 2)   # Thin white center line
    
    # Draw intersections
    for intersection in intersections:
        pygame.draw.circle(screen, BLACK, intersection, 15)
        pygame.draw.circle(screen, (180, 180, 180), intersection, 12)
    
    # Draw start and end points
    if start_point:
        pygame.draw.circle(screen, RED, start_point, 20)
    if end_point:
        pygame.draw.circle(screen, BLUE, end_point, 20)
    
    # Draw route for main car
    if route:
        for i in range(len(route) - 1):
            pygame.draw.line(screen, PURPLE, route[i], route[i+1], 2)
        
        # Highlight current target
        if current_target < len(route):
            pygame.draw.circle(screen, RED, route[current_target], 8)
    
    # Draw white cars
    for car in white_cars:
        draw_car(car.x, car.y, car.angle, WHITE)
    
    # Draw main car (purple)
    if main_car_x is not None and main_car_y is not None:
        draw_car(main_car_x, main_car_y, main_car_angle, PURPLE)
    
    # Draw instructions
    font = pygame.font.SysFont('Arial', 16)
    instructions = [
        "Traffic Simulator",
        "Click red intersection to set start, blue for end",
        "ESC to exit"
    ]
    
    for i, text in enumerate(instructions):
        text_surface = font.render(text, True, WHITE)
        screen.blit(text_surface, (20, 20 + i * 20))

# Update main car position
def update_main_car():
    global main_car_x, main_car_y, main_car_angle, current_target
    
    if main_car_x is None or main_car_y is None or current_target >= len(route):
        return
    
    # Get current target
    target_x, target_y = route[current_target]
    
    # Calculate direction to target
    dx = target_x - main_car_x
    dy = target_y - main_car_y
    target_angle = math.degrees(math.atan2(dy, dx))
    
    # Rotate car towards target (smooth turning)
    angle_diff = (target_angle - main_car_angle + 180) % 360 - 180
    if abs(angle_diff) > 5:
        main_car_angle += angle_diff * 0.1
    else:
        main_car_angle = target_angle
    
    # Move car forward
    main_car_x += math.cos(math.radians(main_car_angle)) * main_car_speed
    main_car_y += math.sin(math.radians(main_car_angle)) * main_car_speed
    
    # Check if reached target
    distance = math.sqrt(dx*dx + dy*dy)
    if distance < 20:
        current_target += 1

# Handle mouse clicks
def handle_click(pos):
    global start_point, end_point, route, main_car_x, main_car_y, main_car_angle, current_target
    
    # Check if clicked near an intersection
    for intersection in intersections:
        dx = pos[0] - intersection[0]
        dy = pos[1] - intersection[1]
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < 25:
            if not start_point:
                # Set start point
                start_point = intersection
                main_car_x, main_car_y = intersection
                main_car_angle = 0
                current_target = 0
                route = []
            elif not end_point:
                # Set end point
                end_point = intersection
                
                # Find path
                path = find_path(start_point, end_point, roads)
                if path:
                    route = path
                    current_target = 1  # Skip the start point
                else:
                    # No path found, just go directly
                    route = [start_point, end_point]
                    current_target = 1
            else:
                # Reset
                start_point = intersection
                end_point = None
                main_car_x, main_car_y = intersection
                main_car_angle = 0
                current_target = 0
                route = []
            break

# Generate random route for white cars
def generate_random_route():
    start = random.choice(intersections)
    end = random.choice(intersections)
    while start == end:  # Make sure start and end are different
        end = random.choice(intersections)
    
    path = find_path(start, end, roads)
    if not path:
        return [start, end]  # Fallback to direct path
    return path

# Create some white cars
def create_white_cars(count=5):
    cars = []
    for _ in range(count):
        route = generate_random_route()
        car = WhiteCar(route[0][0], route[0][1], route)
        cars.append(car)
    return cars

# Generate the map
intersections, roads = generate_grid()

# Create white cars
white_cars = create_white_cars(8)  # Create 8 white cars

# Main game loop
running = True
clock = pygame.time.Clock()

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            handle_click(pygame.mouse.get_pos())
    
    # Update main car position
    update_main_car()
    
    # Update white cars
    for car in white_cars:
        car.update()
    
    # Draw everything
    draw_scene()
    
    # Update the display
    pygame.display.flip()
    clock.tick(60)

# Quit pygame
pygame.quit()
sys.exit() 