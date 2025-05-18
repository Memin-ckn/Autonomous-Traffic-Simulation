#!/usr/bin/env python3

"""
UI Manager component for the traffic simulation.
Handles user interface elements and interaction.
"""

import pygame
import math


class UIManager:
    """
    Handles user interface elements and user interaction for the simulation.
    Manages dropdowns, buttons, and other UI components.
    """
    def __init__(self, width, height):
        """
        Initialize the UI manager
        
        Args:
            width (int): Screen width
            height (int): Screen height
        """
        self.width = width
        self.height = height
        
        # Font initialization
        self.font = pygame.font.SysFont('Arial', int(height / 30))
        
        # Dropdown dimensions
        self.dropdown_width = int(width / 6)
        self.dropdown_height = int(height / 20)
        
        # Grid size dropdown
        self.grid_sizes = [3, 4, 5, 6, 7, 8]
        self.current_grid_size = 4  # Default grid size
        self.dropdown_open = False
        
        # Car count dropdown
        self.car_counts = [1, 2, 3, 4, 5, 6, 7, 8]
        self.current_car_count = 4  # Default car count
        self.car_dropdown_open = False
        
        # Instructions
        self.show_instructions = True
        self.instructions = [
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
    
    def get_grid_dropdown_rect(self):
        """
        Get the rectangle for the grid size dropdown
        
        Returns:
            pygame.Rect: Rectangle for the grid size dropdown
        """
        return pygame.Rect(
            int(self.width * 0.05),
            int(self.height * 0.05),
            self.dropdown_width,
            self.dropdown_height
        )
    
    def get_car_dropdown_rect(self):
        """
        Get the rectangle for the car count dropdown
        
        Returns:
            pygame.Rect: Rectangle for the car count dropdown
        """
        return pygame.Rect(
            int(self.width * 0.05) + self.dropdown_width + 20,
            int(self.height * 0.05),
            self.dropdown_width,
            self.dropdown_height
        )
    
    def get_grid_option_rect(self, index):
        """
        Get the rectangle for a grid size option
        
        Args:
            index (int): Index of the option in the grid_sizes list
            
        Returns:
            pygame.Rect: Rectangle for the option
        """
        grid_dropdown_rect = self.get_grid_dropdown_rect()
        return pygame.Rect(
            grid_dropdown_rect.x,
            grid_dropdown_rect.y + (index + 1) * self.dropdown_height,
            self.dropdown_width,
            self.dropdown_height
        )
    
    def get_car_option_rect(self, index):
        """
        Get the rectangle for a car count option
        
        Args:
            index (int): Index of the option in the car_counts list
            
        Returns:
            pygame.Rect: Rectangle for the option
        """
        car_dropdown_rect = self.get_car_dropdown_rect()
        return pygame.Rect(
            car_dropdown_rect.x,
            car_dropdown_rect.y + (index + 1) * self.dropdown_height,
            self.dropdown_width,
            self.dropdown_height
        )
    
    def handle_click(self, pos, intersections, handle_intersection_click):
        """
        Handle a mouse click on the UI
        
        Args:
            pos (tuple): Mouse position (x, y)
            intersections (list): List of intersection points
            handle_intersection_click (callable): Function to call when an intersection is clicked
            
        Returns:
            tuple: (grid_size_changed, car_count_changed, need_update)
            - grid_size_changed (bool): Whether the grid size was changed
            - car_count_changed (bool): Whether the car count was changed
            - need_update (bool): Whether the UI needs updating
        """
        # Check for grid size dropdown menu clicks
        grid_dropdown_rect = self.get_grid_dropdown_rect()
        if grid_dropdown_rect.collidepoint(pos):
            self.dropdown_open = not self.dropdown_open
            self.car_dropdown_open = False  # Close other dropdown
            return False, False, False
            
        # Check for car count dropdown clicks
        car_dropdown_rect = self.get_car_dropdown_rect()
        if car_dropdown_rect.collidepoint(pos):
            self.car_dropdown_open = not self.car_dropdown_open
            self.dropdown_open = False  # Close other dropdown
            return False, False, False
            
        # If grid size dropdown is open
        if self.dropdown_open:
            for i, size in enumerate(self.grid_sizes):
                option_rect = self.get_grid_option_rect(i)
                if option_rect.collidepoint(pos):
                    if size != self.current_grid_size:
                        self.current_grid_size = size
                        self.dropdown_open = False
                        return True, False, True
                    self.dropdown_open = False
                    return False, False, True
        
        # If car count dropdown is open
        if self.car_dropdown_open:
            for i, count in enumerate(self.car_counts):
                option_rect = self.get_car_option_rect(i)
                if option_rect.collidepoint(pos):
                    if count != self.current_car_count:
                        self.current_car_count = count
                        self.car_dropdown_open = False
                        return False, True, True
                    self.car_dropdown_open = False
                    return False, False, True
        
        # Check for intersection clicks
        for intersection in intersections:
            # Distance to intersection
            dx = pos[0] - intersection[0]
            dy = pos[1] - intersection[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            # If clicked on an intersection
            if distance < 15:
                handle_intersection_click(intersection)
                return False, False, True
        
        # Click was on empty space, just close dropdowns
        if self.dropdown_open or self.car_dropdown_open:
            self.dropdown_open = False
            self.car_dropdown_open = False
            return False, False, True
            
        return False, False, False
    
    def get_grid_size(self):
        """
        Get the current grid size
        
        Returns:
            int: Current grid size
        """
        return self.current_grid_size
    
    def get_car_count(self):
        """
        Get the current car count
        
        Returns:
            int: Current car count
        """
        return self.current_car_count
    
    def get_instructions(self):
        """
        Get the instruction text
        
        Returns:
            list: List of instruction strings
        """
        return self.instructions
    
    def is_grid_dropdown_open(self):
        """
        Check if the grid size dropdown is open
        
        Returns:
            bool: True if open, False otherwise
        """
        return self.dropdown_open
    
    def is_car_dropdown_open(self):
        """
        Check if the car count dropdown is open
        
        Returns:
            bool: True if open, False otherwise
        """
        return self.car_dropdown_open
    
    def get_grid_dropdown_text(self):
        """
        Get the text for the grid size dropdown
        
        Returns:
            str: Text for the grid size dropdown
        """
        return f"Grid Size: {self.current_grid_size}x{self.current_grid_size}"
    
    def get_car_dropdown_text(self):
        """
        Get the text for the car count dropdown
        
        Returns:
            str: Text for the car count dropdown
        """
        return f"Car Count: {self.current_car_count}" 