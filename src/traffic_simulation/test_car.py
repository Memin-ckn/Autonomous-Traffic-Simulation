#!/usr/bin/env python3

import pygame
import sys
import math

# Initialize pygame
pygame.init()

# Create a fullscreen window
screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
width, height = screen.get_size()
pygame.display.set_caption("Test Car")

# Function to draw a huge car
def draw_car(screen, pos, heading=0, color=(255, 0, 255)):
    x, y = pos
    
    # Define car dimensions as percentages of screen height
    car_length = int(height * 0.2)  # 20% of screen height
    car_width = int(height * 0.1)   # 10% of screen height
    
    # Calculate trigonometric values once
    cos_h = math.cos(heading)
    sin_h = math.sin(heading)
    
    # Calculate corners
    half_length = car_length / 2
    half_width = car_width / 2
    
    # Draw black border
    border_width = 8
    border_corners = [
        (x + (half_length + border_width) * cos_h - (half_width + border_width) * sin_h,
         y + (half_length + border_width) * sin_h + (half_width + border_width) * cos_h),
        (x + (half_length + border_width) * cos_h + (half_width + border_width) * sin_h,
         y + (half_length + border_width) * sin_h - (half_width + border_width) * cos_h),
        (x - (half_length + border_width) * cos_h + (half_width + border_width) * sin_h,
         y - (half_length + border_width) * sin_h - (half_width + border_width) * cos_h),
        (x - (half_length + border_width) * cos_h - (half_width + border_width) * sin_h,
         y - (half_length + border_width) * sin_h + (half_width + border_width) * cos_h)
    ]
    pygame.draw.polygon(screen, (0, 0, 0), border_corners)
    
    # Draw car body
    corners = [
        (x + half_length * cos_h - half_width * sin_h,
         y + half_length * sin_h + half_width * cos_h),
        (x + half_length * cos_h + half_width * sin_h,
         y + half_length * sin_h - half_width * cos_h),
        (x - half_length * cos_h + half_width * sin_h,
         y - half_length * sin_h - half_width * cos_h),
        (x - half_length * cos_h - half_width * sin_h,
         y - half_length * sin_h + half_width * cos_h)
    ]
    pygame.draw.polygon(screen, color, corners)
    
    # Draw inner area
    inner_length = car_length * 0.7
    inner_width = car_width * 0.7
    half_inner_length = inner_length / 2
    half_inner_width = inner_width / 2
    
    inner_corners = [
        (x + half_inner_length * cos_h - half_inner_width * sin_h,
         y + half_inner_length * sin_h + half_inner_width * cos_h),
        (x + half_inner_length * cos_h + half_inner_width * sin_h,
         y + half_inner_length * sin_h - half_inner_width * cos_h),
        (x - half_inner_length * cos_h + half_inner_width * sin_h,
         y - half_inner_length * sin_h - half_inner_width * cos_h),
        (x - half_inner_length * cos_h - half_inner_width * sin_h,
         y - half_inner_length * sin_h + half_inner_width * cos_h)
    ]
    pygame.draw.polygon(screen, (255, 150, 255), inner_corners)  # Lighter color
    
    # Draw front marker (red circle)
    front_x = x + half_length * cos_h
    front_y = y + half_length * sin_h
    pygame.draw.circle(screen, (255, 0, 0), (int(front_x), int(front_y)), int(car_width/3))
    
    # Draw debug text
    font = pygame.font.SysFont('Arial', 30)
    text = font.render(f"CAR SIZE: {car_length}x{car_width}", True, (255, 255, 255))
    screen.blit(text, (x - 150, y - half_width - 40))

def draw_simple_car(screen, pos):
    """Draw a simple rectangular car"""
    x, y = pos
    car_length = int(height * 0.2)  # 20% of screen height
    car_width = int(height * 0.1)   # 10% of screen height
    
    # Black border
    pygame.draw.rect(screen, (0, 0, 0), 
                   (x - car_length//2 - 4, 
                    y - car_width//2 - 4,
                    car_length + 8, 
                    car_width + 8))
    
    # Purple body
    pygame.draw.rect(screen, (255, 0, 255), 
                   (x - car_length//2, 
                    y - car_width//2,
                    car_length, 
                    car_width))
    
    # Red front circle
    pygame.draw.circle(screen, (255, 0, 0), 
                     (x + car_length//2, y), 
                     car_width//3)
    
    # Text label
    font = pygame.font.SysFont('Arial', 30)
    text = font.render(f"SIMPLE CAR: {car_length}x{car_width}", True, (255, 255, 255))
    screen.blit(text, (x - 150, y - car_width//2 - 40))

# Main game loop
running = True
clock = pygame.time.Clock()
heading = 0

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                running = False
                
    # Clear screen
    screen.fill((34, 139, 34))  # Green background
    
    # Update heading for rotation
    heading += 0.01
    
    # Draw both car types
    draw_car(screen, (width // 2, height // 3), heading)
    draw_simple_car(screen, (width // 2, height * 2 // 3))
    
    # Instructions
    font = pygame.font.SysFont('Arial', 24)
    text = font.render("Press ESC to exit", True, (255, 255, 255))
    screen.blit(text, (20, 20))
    
    # Update display
    pygame.display.flip()
    clock.tick(60)

# Quit pygame
pygame.quit()
sys.exit() 