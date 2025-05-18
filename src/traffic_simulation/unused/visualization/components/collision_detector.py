#!/usr/bin/env python3

"""
Collision detection component for the traffic simulation.
Handles collision detection and prediction between vehicles.
"""

import math


class CollisionDetector:
    """
    Handles collision detection and prediction between vehicles in the simulation.
    Uses Separating Axis Theorem (SAT) for accurate collision detection.
    """
    def __init__(self, car_width, car_height, road_width):
        """
        Initialize the collision detector with vehicle dimensions
        
        Args:
            car_width (int): Width of cars in the simulation
            car_height (int): Height of cars in the simulation
            road_width (int): Width of roads (for proximity calculations)
        """
        self.car_width = car_width
        self.car_height = car_height
        self.road_width = road_width
        
        # Detection zone parameters
        self.safety_zone_size = 1.5  # Multiplier for car size in safety zone
        self.detection_zone_size = 3.5  # Multiplier for car size in detection zone
        self.detection_range = road_width * 10  # Large detection radius
    
    def get_car_corners(self, x, y, angle, safety_margin=1.0):
        """
        Calculate the four corners of a car for collision detection
        
        Args:
            x (float): X coordinate of car center
            y (float): Y coordinate of car center
            angle (float): Angle in degrees
            safety_margin (float): Multiplier for car size (1.0 = normal size)
            
        Returns:
            list: Four corner points of the car
        """
        rad_angle = math.radians(angle)
        
        # Calculate corners (unrotated)
        half_width = (self.car_width * safety_margin) // 2
        half_height = (self.car_height * safety_margin) // 2
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
        """
        Check if two cars are colliding using Separating Axis Theorem (SAT)
        Also considers traffic lanes based on direction of travel
        
        Args:
            car1: First vehicle object with x, y, angle attributes
            car2: Second vehicle object with x, y, angle attributes
            
        Returns:
            bool: True if collision detected, False otherwise
        """
        # First, check if cars are traveling in significantly different directions
        # If so, we can assume they're in different lanes and won't collide
        if hasattr(car1, 'angle') and hasattr(car2, 'angle'):
            angle_diff = abs((car1.angle - car2.angle + 180) % 360 - 180)
            
            # If cars are moving nearly perpendicular (75-105 degrees) 
            # or opposite (165-195 degrees), consider them in different lanes
            if (75 < angle_diff < 105) or angle_diff > 165:
                # Cars in perpendicular or opposite directions - assume different lanes
                # But still check if they're very close (crossing an intersection)
                dx = car1.x - car2.x
                dy = car1.y - car2.y
                distance = math.sqrt(dx*dx + dy*dy)
                
                # Only ignore lane-based pass-through if they're not very close
                if distance > self.car_width * 1.5:
                    return False
        
        # Use a larger safety margin for the purple car (SmartCar) to keep it further away
        # from white cars
        purple_safety_margin = 1.5  # Increase the collision boundary by 50%
        
        # Get corners of both cars
        # If car1 is the purple car (SmartCar), use the larger safety margin
        if hasattr(car1, 'id') and car1.id == "purple_car":
            corners1 = self.get_car_corners(car1.x, car1.y, car1.angle, safety_margin=purple_safety_margin)
        else:
            corners1 = self.get_car_corners(car1.x, car1.y, car1.angle)
            
        # Always use normal hitbox for white cars
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
    
    def predict_collision(self, car1, car2, time_steps=15):
        """
        Predict if two cars will collide within the next few time steps
        
        Args:
            car1: First vehicle object with x, y, angle attributes
            car2: Second vehicle object with x, y, angle attributes
            time_steps (int): Number of time steps to look ahead
            
        Returns:
            tuple: (will_collide, time_to_collision)
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
                def __init__(self, x, y, angle, car_id=None):
                    self.x = x
                    self.y = y
                    self.angle = angle
                    self.id = car_id
            
            # Pass the car's ID if it's the purple car
            future_car1 = TempCar(future_x1, future_y1, angle1, 
                                car_id="purple_car" if hasattr(car1, 'id') and car1.id == "purple_car" else None)
            future_car2 = TempCar(future_x2, future_y2, angle2)
            
            if self.check_collision(future_car1, future_car2):
                return True, step
        
        return False, -1
    
    def check_all_collisions(self, smart_car, white_cars, collision_strategy=None):
        """
        Check for collisions between the main car and all other cars
        
        Args:
            smart_car: The main vehicle to check collisions for
            white_cars (list): List of other vehicles
            collision_strategy: Enum class for collision strategies
            
        Returns:
            tuple: (immediate_collisions, predicted_collisions)
        """
        if smart_car is None:
            return [], []
            
        collisions = []  # List of (car1, car2) for immediate collisions
        predicted_collisions = []  # List of (car1, car2, time) for predicted collisions
        
        # Check main car against all white cars
        for car in white_cars:
            # Check for immediate collision
            if self.check_collision(smart_car, car):
                collisions.append((smart_car, car))
                
                # Set collision risk flag
                smart_car.collision_risk = True
                # Emergency stop will be handled by gradual deceleration
                if collision_strategy:
                    smart_car.current_strategy = collision_strategy.YIELD
            
            # Check for predicted collisions
            else:
                # Check if car is within detection range
                dx = car.x - smart_car.x
                dy = car.y - smart_car.y
                distance = math.sqrt(dx*dx + dy*dy)
                
                # Only consider cars within the detection range
                if distance <= self.detection_range:
                    will_collide, time_to_collision = self.predict_collision(smart_car, car)
                    if will_collide:
                        predicted_collisions.append((smart_car, car, time_to_collision))
                        
                        # Set appropriate strategy based on time to collision
                        smart_car.collision_risk = True
                        
                        if collision_strategy:
                            # Very close collision - yield or slow down
                            if time_to_collision <= 5:
                                smart_car.current_strategy = collision_strategy.SLOW_DOWN
        
        return collisions, predicted_collisions 