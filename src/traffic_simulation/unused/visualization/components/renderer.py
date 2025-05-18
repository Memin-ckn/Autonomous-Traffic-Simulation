#!/usr/bin/env python3

"""
Renderer component for the traffic simulation.
Handles all drawing operations for the simulation.
"""

import pygame
import math


class SimulationRenderer:
    """
    Handles all rendering operations for the traffic simulation.
    Separates drawing logic from the main simulation logic.
    """
    def __init__(self, screen, width, height):
        """
        Initialize the renderer with the pygame screen and dimensions
        
        Args:
            screen (pygame.Surface): The pygame screen surface to draw on
            width (int): Screen width
            height (int): Screen height
        """
        self.screen = screen
        self.width = width
        self.height = height
        
        # Define colors
        self.GREEN = (34, 139, 34)      # Background
        self.BLACK = (0, 0, 0)          # Road, outlines
        self.WHITE = (255, 255, 255)    # Road lines and cars
        self.RED = (255, 0, 0)          # Start point
        self.BLUE = (0, 0, 255)         # End point
        self.PURPLE = (255, 0, 255)     # Main car color
        self.YELLOW = (255, 255, 0)     # Highlights
        self.ORANGE = (255, 165, 0)     # Warning color
        
        # Font initialization
        self.font = pygame.font.SysFont('Arial', int(height / 30))
        self.small_font = pygame.font.SysFont('Arial', 16)
        self.medium_font = pygame.font.SysFont('Arial', 18, bold=True)
    
    def clear_screen(self):
        """Clear the screen with the background color."""
        self.screen.fill(self.GREEN)
    
    def draw_road(self, start, end, road_width):
        """
        Draw a road segment between two points
        
        Args:
            start (tuple): Start point (x, y)
            end (tuple): End point (x, y)
            road_width (int): Width of the road
        """
        # Draw thick black road
        pygame.draw.line(self.screen, self.BLACK, start, end, road_width)
        # Draw thin white center line
        pygame.draw.line(self.screen, self.WHITE, start, end, 2)
    
    def draw_intersection(self, position):
        """
        Draw an intersection at the specified position
        
        Args:
            position (tuple): Position (x, y) of the intersection
        """
        pygame.draw.circle(self.screen, self.BLACK, position, 15)
        pygame.draw.circle(self.screen, (180, 180, 180), position, 12)
    
    def draw_waypoint(self, position, color, radius=6):
        """
        Draw a waypoint at the specified position
        
        Args:
            position (tuple): Position (x, y) of the waypoint
            color (tuple): RGB color tuple
            radius (int): Radius of the waypoint circle
        """
        pygame.draw.circle(self.screen, color, position, radius)
    
    def draw_car(self, x, y, angle, color, car_width, car_height):
        """
        Draw a car at the specified position and orientation
        
        Args:
            x (float): X coordinate
            y (float): Y coordinate
            angle (float): Angle in degrees
            color (tuple): RGB color tuple
            car_width (int): Width of the car
            car_height (int): Height of the car
        """
        # Convert angle from degrees to radians
        rad_angle = math.radians(angle)
        
        # Calculate car center
        center_x, center_y = x, y
        
        # Calculate corners (unrotated)
        half_width = car_width // 2
        half_height = car_height // 2
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
        
        # Draw front indicator (red circle)
        front_x = center_x + half_width * math.cos(rad_angle)
        front_y = center_y + half_width * math.sin(rad_angle)
        pygame.draw.circle(self.screen, self.RED, (int(front_x), int(front_y)), 8)
    
    def draw_route(self, route, color, line_width=2):
        """
        Draw a route as a series of connected lines
        
        Args:
            route (list): List of (x, y) points in the route
            color (tuple): RGB color tuple
            line_width (int): Width of the route lines
        """
        if len(route) < 2:
            return
            
        for i in range(len(route) - 1):
            pygame.draw.line(self.screen, color, route[i], route[i+1], line_width)
    
    def draw_dropdown(self, rect, text, is_open=False, options=None, option_values=None):
        """
        Draw a dropdown menu
        
        Args:
            rect (pygame.Rect): Rectangle for the dropdown
            text (str): Text to display
            is_open (bool): Whether the dropdown is open
            options (list): List of option texts to display if open
            option_values (list): List of values corresponding to options
        """
        # Draw dropdown box
        pygame.draw.rect(self.screen, (220, 220, 220), rect)
        pygame.draw.rect(self.screen, self.BLACK, rect, 2)
        text_surface = self.font.render(text, True, self.BLACK)
        self.screen.blit(text_surface, (rect.x + 10, rect.y + 5))
        
        # Draw options if open
        if is_open and options:
            for i, option in enumerate(options):
                option_rect = pygame.Rect(
                    rect.x,
                    rect.y + (i + 1) * rect.height,
                    rect.width,
                    rect.height
                )
                pygame.draw.rect(self.screen, (200, 200, 200), option_rect)
                pygame.draw.rect(self.screen, self.BLACK, option_rect, 1)
                option_text = self.font.render(str(option), True, self.BLACK)
                self.screen.blit(option_text, (option_rect.x + 10, option_rect.y + 5))
                
    def draw_instructions(self, instructions):
        """
        Draw instruction text on the screen
        
        Args:
            instructions (list): List of instruction strings
        """
        for i, text in enumerate(instructions):
            text_surface = self.small_font.render(text, True, self.WHITE)
            self.screen.blit(text_surface, (20, 20 + i * 20))
    
    def draw_status_text(self, text, color, position):
        """
        Draw status text at the specified position
        
        Args:
            text (str): Text to display
            color (tuple): RGB color tuple
            position (tuple): Position (x, y) to draw the text
        """
        text_surface = self.medium_font.render(text, True, color)
        self.screen.blit(text_surface, position)
    
    def draw_car_indicator(self, x, y, indicator_type):
        """
        Draw an indicator above a car
        
        Args:
            x (float): X coordinate
            y (float): Y coordinate
            indicator_type (str): Type of indicator ('SLOW', 'STOP', 'REROUTE', 'TURN')
        """
        indicator_y = y - 40  # Position above car
        
        if indicator_type == 'SLOW':
            # Orange slow down indicator
            pygame.draw.circle(self.screen, self.ORANGE, (int(x), int(indicator_y)), 15)
            text = self.small_font.render("SLOW", True, self.BLACK)
            self.screen.blit(text, (x - 15, indicator_y - 7))
            
        elif indicator_type == 'STOP':
            # Red stop indicator
            pygame.draw.circle(self.screen, self.RED, (int(x), int(indicator_y)), 15)
            text = self.small_font.render("STOP", True, self.WHITE)
            self.screen.blit(text, (x - 15, indicator_y - 7))
            
        elif indicator_type == 'REROUTE':
            # Blue reroute indicator
            pygame.draw.circle(self.screen, (0, 100, 255), (int(x), int(indicator_y)), 15)
            text = self.small_font.render("REROUTE", True, self.WHITE)
            self.screen.blit(text, (x - 25, indicator_y - 7))
            
        elif indicator_type == 'TURN':
            # Orange turn indicator
            pygame.draw.circle(self.screen, (255, 140, 0), (int(x), int(indicator_y)), 15)
            text = self.small_font.render("TURN", True, self.BLACK)
            self.screen.blit(text, (x - 15, indicator_y - 7))
    
    def draw_detection_zone(self, x, y, radius):
        """
        Draw a semi-transparent detection zone
        
        Args:
            x (float): X coordinate of center
            y (float): Y coordinate of center
            radius (float): Radius of the detection zone
        """
        # Create a surface with per-pixel alpha
        surface = pygame.Surface((radius*2, radius*2), pygame.SRCALPHA)
        pygame.draw.circle(surface, (0, 0, 255, 20), (radius, radius), radius)
        self.screen.blit(surface, (x - radius, y - radius))
    
    def draw_ghost_danger_zone(self, x, y, radius):
        """
        Draw a semi-transparent danger zone around ghosts
        
        Args:
            x (float): X coordinate of center
            y (float): Y coordinate of center
            radius (float): Radius of the danger zone
        """
        # Create a surface with per-pixel alpha
        surface = pygame.Surface((radius*2, radius*2), pygame.SRCALPHA)
        pygame.draw.circle(surface, (255, 0, 0, 40), (radius, radius), radius)
        self.screen.blit(surface, (x - radius, y - radius))
    
    def draw_metrics(self, safety_score, efficiency_score, combined_score):
        """
        Draw route metrics on the screen
        
        Args:
            safety_score (float): Safety score (0-100)
            efficiency_score (float): Efficiency score (0-100) 
            combined_score (float): Combined score (0-100)
        """
        # Set colors based on scores
        if safety_score >= 80:
            safety_color = (0, 200, 0)  # Green for safe
        elif safety_score >= 50:
            safety_color = (255, 165, 0)  # Orange for caution
        else:
            safety_color = (255, 0, 0)  # Red for dangerous
        
        if efficiency_score >= 90:
            efficiency_color = (0, 200, 0)  # Green for efficient
        elif efficiency_score >= 70:
            efficiency_color = (255, 165, 0)  # Orange for somewhat efficient
        else:
            efficiency_color = (255, 0, 0)  # Red for inefficient
            
        if combined_score >= 80:
            combined_color = (0, 200, 0)  # Green for good
        elif combined_score >= 60:
            combined_color = (255, 165, 0)  # Orange for average
        else:
            combined_color = (255, 0, 0)  # Red for poor
        
        # Display safety score
        safety_text = f"Route Safety: {safety_score:.1f}/100"
        safety_surface = self.medium_font.render(safety_text, True, safety_color)
        self.screen.blit(safety_surface, (self.width - safety_surface.get_width() - 20, 50))
        
        # Display efficiency score
        efficiency_text = f"Route Efficiency: {efficiency_score:.1f}%"
        efficiency_surface = self.medium_font.render(efficiency_text, True, efficiency_color)
        self.screen.blit(efficiency_surface, (self.width - efficiency_surface.get_width() - 20, 80))
        
        # Display combined score
        combined_text = f"Overall Route Quality: {combined_score:.1f}/100"
        combined_surface = self.medium_font.render(combined_text, True, combined_color)
        self.screen.blit(combined_surface, (self.width - combined_surface.get_width() - 20, 110))
    
    def draw_car_corners(self, corners, color, line_width=2):
        """
        Draw the corners of a car (for hitbox visualization)
        
        Args:
            corners (list): List of (x, y) corner points
            color (tuple): RGB color tuple
            line_width (int): Width of the lines
        """
        pygame.draw.lines(self.screen, color, True, corners, line_width) 