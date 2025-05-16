#!/usr/bin/env python3

import pygame
import sys

# Initialize pygame
pygame.init()

# Create a window (not fullscreen for easier testing)
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Simplest Car Test")

# Function to draw a car
def draw_car():
    # Draw a HUGE rectangle for the car body
    car_x = WIDTH // 2 - 150  # Left edge of car
    car_y = HEIGHT // 2 - 75  # Top edge of car
    car_width = 300  # Very wide
    car_height = 150  # Very tall
    
    # Black outline
    pygame.draw.rect(screen, (0, 0, 0), (car_x-5, car_y-5, car_width+10, car_height+10))
    
    # Magenta car body
    pygame.draw.rect(screen, (255, 0, 255), (car_x, car_y, car_width, car_height))
    
    # Red circle at front
    pygame.draw.circle(screen, (255, 0, 0), (car_x + car_width, car_y + car_height//2), 40)
    
    # Yellow corner markers
    pygame.draw.circle(screen, (255, 255, 0), (car_x, car_y), 20)  # top left
    pygame.draw.circle(screen, (255, 255, 0), (car_x + car_width, car_y), 20)  # top right
    pygame.draw.circle(screen, (255, 255, 0), (car_x, car_y + car_height), 20)  # bottom left
    pygame.draw.circle(screen, (255, 255, 0), (car_x + car_width, car_y + car_height), 20)  # bottom right

# Main loop
running = True
clock = pygame.time.Clock()

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                running = False
    
    # Clear screen
    screen.fill((50, 150, 50))
    
    # Draw car
    draw_car()
    
    # Draw instructions
    font = pygame.font.SysFont('Arial', 24)
    text = font.render("HUGE CAR TEST - Press ESC to exit", True, (255, 255, 255))
    screen.blit(text, (20, 20))
    
    # Text showing car size
    size_text = font.render("Car size: 300x150 pixels", True, (255, 255, 255))
    screen.blit(size_text, (20, 50))
    
    # Update display
    pygame.display.flip()
    clock.tick(60)

# Quit
pygame.quit()
sys.exit() 