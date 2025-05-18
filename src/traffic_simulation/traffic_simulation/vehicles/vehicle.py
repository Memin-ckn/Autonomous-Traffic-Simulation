import math
import random
import uuid
from typing import List, Tuple, Dict, Optional
import rclpy
from rclpy.node import Node

from traffic_simulation.sensors.sensor import Lidar, Radar, SensorReading
from traffic_simulation.communication.iot import IoTCommunicator, VehicleData
from traffic_simulation.planning.collision_avoidance import CollisionAvoidance, CollisionStrategy


class Vehicle:
    """Base vehicle class with common properties"""
    def __init__(self, x: float, y: float, route: List[Tuple[float, float]], node: Optional[Node] = None):
        self.id = f"vehicle_{uuid.uuid4().hex[:8]}"  # Unique ID
        self.x = x
        self.y = y
        self.angle = 0
        self.speed = random.uniform(1.0, 3.0)
        self.default_speed = self.speed
        self.route = route
        self.current_target = 1  # Skip start point
        self.node = node
        
        # Calculate velocity components
        self.vx = 0
        self.vy = 0
        
        # Vehicle dimensions
        self.width = 60
        self.height = 30
        
        # Collision risk status
        self.collision_risk = False
    
    def update(self, all_vehicles: List["Vehicle"] = None):
        """Update vehicle position based on current route"""
        # Check if reached end of route
        if self.current_target >= len(self.route):
            # Route complete - don't move
            return
        
        # Get current target
        target_x, target_y = self.route[self.current_target]
        
        # Calculate direction to target
        dx = target_x - self.x
        dy = target_y - self.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Only update angle if distance to target is significant
        if distance > 5:
            target_angle = math.degrees(math.atan2(dy, dx))
            
            # Rotate car towards target (smooth turning)
            angle_diff = (target_angle - self.angle + 180) % 360 - 180
            if abs(angle_diff) > 5:
                self.angle += angle_diff * 0.1
            else:
                self.angle = target_angle
        
        # Calculate velocity components
        self.vx = math.cos(math.radians(self.angle)) * self.speed
        self.vy = math.sin(math.radians(self.angle)) * self.speed
        
        # Move car forward
        self.x += self.vx
        self.y += self.vy
        
        # Check if reached target
        if distance < 20:
            self.current_target += 1
            
    def get_position(self) -> Tuple[float, float]:
        """Get current position"""
        return (self.x, self.y)
    
    def get_corners(self) -> List[Tuple[float, float]]:
        """Get the four corners of the vehicle for collision detection"""
        # Convert angle to radians
        rad_angle = math.radians(self.angle)
        
        # Calculate corners (unrotated)
        half_width = self.width // 2
        half_height = self.height // 2
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
            rotated_corners.append((self.x + rotated_x, self.y + rotated_y))
            
        return rotated_corners


class WhiteCar(Vehicle):
    """Fully autonomous car with sensors, IoT, and collision avoidance"""
    def __init__(self, x: float, y: float, route: List[Tuple[float, float]], node: Optional[Node] = None, grid_size: int = 4, road_width: float = 40.0):
        super().__init__(x, y, route, node)
        self.speed = random.uniform(1.0, 2.5)
        self.default_speed = self.speed
        self.vehicle_type = "smart"
        self.color = (255, 255, 255)  # White
        self.id = f"whitecar_{uuid.uuid4().hex[:8]}"

        # Initialize sensors
        self.lidar = Lidar(self.id, range_max=150.0, node=node)
        self.radar = Radar(self.id, range_max=200.0, node=node)

        # Initialize IoT communication
        self.iot = IoTCommunicator(self.id, node)
        self.other_vehicles: Dict[str, VehicleData] = {}

        # Initialize collision avoidance system
        self.collision_avoidance = CollisionAvoidance(
            self.id, grid_size=grid_size, road_width=road_width, default_speed=self.default_speed
        )

        # Collision flags and tracking
        self.collision_risk = False
        self.yield_timer = 0
        self.current_strategy = CollisionStrategy.SLOW_DOWN
        self.original_route = route.copy() if route else []
        self.is_rerouting = False
        self.intersections = []
        self.roads = []

    def set_map_data(self, intersections: List[Tuple[float, float]], 
                    roads: List[Tuple[Tuple[float, float], Tuple[float, float]]]):
        """Set map data for path planning"""
        self.intersections = intersections
        self.roads = roads

    def update(self, all_vehicles: List[Vehicle] = None):
        """
        Update vehicle with sensor readings and collision avoidance
        all_vehicles: List of all vehicles in the simulation for sensor detection
        """
        if not all_vehicles:
            all_vehicles = []

        # Reset speed to default unless we're yielding
        if self.current_strategy != CollisionStrategy.YIELD or self.yield_timer <= 0:
            self.speed = self.default_speed

        # Check if we're yielding
        if self.current_strategy == CollisionStrategy.YIELD and self.yield_timer > 0:
            # Continue yielding for a bit
            self.yield_timer -= 1
            self.speed = 0.0
            return

        # Sensor updates
        if hasattr(self, 'lidar') and self.lidar:
            lidar_readings = self.lidar.detect_objects(
                all_vehicles, (self.x, self.y), self.angle
            )
            self.lidar.publish_readings()
        else:
            lidar_readings = []

        if hasattr(self, 'radar') and self.radar:
            radar_readings = self.radar.detect_objects(
                all_vehicles, (self.x, self.y), self.angle
            )
            self.radar.publish_readings()
        else:
            radar_readings = []

        # Share data via IoT if available
        if hasattr(self, 'iot') and self.iot:
            # Create vehicle data
            now = self.node.get_clock().now().nanoseconds / 1e9 if self.node else 0.0
            data = VehicleData(
                vehicle_id=self.id,
                position=(self.x, self.y),
                velocity=(self.vx, self.vy),
                heading=self.angle,
                route=self.route,
                current_target=self.current_target,
                timestamp=now,
                vehicle_type=self.vehicle_type
            )

            # Broadcast data
            self.iot.broadcast_data(data)

            # Get other vehicle data
            self.other_vehicles = self.iot.known_vehicles

        # Process sensor data for immediate collision avoidance
        if hasattr(self, 'collision_avoidance') and self.collision_avoidance:
            collision_risk, recommended_speed = self.collision_avoidance.process_sensor_data(
                lidar_readings, radar_readings
            )

            # Check route conflicts if we have IoT data about other vehicles
            if self.other_vehicles and self.route and self.current_target < len(self.route):
                # Filter other_vehicles to only include white cars
                white_car_data = {
                    vid: data for vid, data in self.other_vehicles.items() 
                    if vid.startswith('whitecar_') and vid != self.id
                }

                if white_car_data:
                    strategy, target_idx, conflict_speed = self.collision_avoidance.check_route_conflicts(
                        self.route, white_car_data, self.current_target
                    )

                    # Apply recommended strategy
                    self.current_strategy = strategy

                    if strategy == CollisionStrategy.SLOW_DOWN:
                        # Take the more conservative speed adjustment
                        self.speed = min(recommended_speed, conflict_speed)

                    elif strategy == CollisionStrategy.YIELD:
                        # Stop and wait
                        self.speed = 0.0
                        self.yield_timer = 60  # Wait for ~2 seconds (assuming 30 FPS)

                    elif strategy == CollisionStrategy.REROUTE and not self.is_rerouting:
                        # Find alternative route
                        self.is_rerouting = True
                        if self.intersections and self.roads:
                            # Find conflict point
                            if self.current_target < len(self.route):
                                conflict_point = self.route[self.current_target]
                                # Current position as start, original goal as end
                                start = (self.x, self.y)
                                end = self.original_route[-1] if self.original_route else self.route[-1]

                                # Get new route
                                new_route = self.collision_avoidance.find_alternative_route(
                                    self.intersections, self.roads, start, end, conflict_point
                                )

                                if new_route and len(new_route) > 1:
                                    # Replace route from current position onwards
                                    self.route = [start] + new_route
                                    self.current_target = 1  # Start from next point

                        self.is_rerouting = False
            else:
                # If no IoT data or no white cars nearby, just use sensor-based speed adjustment
                self.speed = recommended_speed

            # Update collision risk status for visualization
            self.collision_risk = collision_risk

        # Basic movement update (same as regular vehicle)
        if self.current_target < len(self.route):
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

            # Update velocity components based on current speed
            self.vx = math.cos(math.radians(self.angle)) * self.speed
            self.vy = math.sin(math.radians(self.angle)) * self.speed

            # Move car forward
            self.x += self.vx
            self.y += self.vy

            # Check if reached target
            distance = math.sqrt(dx*dx + dy*dy)
            if distance < 20:
                self.current_target += 1


class SmartCar(Vehicle):
    """
    Advanced car with sensors, IoT communication, and collision avoidance
    This represents the purple car with all capabilities
    """
    def __init__(self, x: float, y: float, route: List[Tuple[float, float]],
                node: Optional[Node] = None, grid_size: int = 4, road_width: float = 40.0):
        super().__init__(x, y, route, node)
        self.id = "purple_car"  # Fixed ID for the main car
        self.speed = 3.0
        self.default_speed = self.speed
        self.color = (255, 0, 255)  # Purple color
        self.vehicle_type = "smart"
        
        # Initialize sensors
        self.lidar = Lidar(self.id, range_max=150.0, node=node)
        self.radar = Radar(self.id, range_max=200.0, node=node)
        
        # Initialize IoT communication
        self.iot = IoTCommunicator(self.id, node)
        self.other_vehicles: Dict[str, VehicleData] = {}
        
        # Initialize collision avoidance system
        self.collision_avoidance = CollisionAvoidance(
            self.id, grid_size=grid_size, road_width=road_width, default_speed=self.default_speed
        )
        
        # Collision flags and tracking
        self.collision_risk = False
        self.yield_timer = 0
        self.current_strategy = CollisionStrategy.SLOW_DOWN
        self.original_route = route.copy() if route else []
        self.is_rerouting = False
        self.intersections = []
        self.roads = []
    
    def set_map_data(self, intersections: List[Tuple[float, float]], 
                    roads: List[Tuple[Tuple[float, float], Tuple[float, float]]]):
        """Set map data for path planning"""
        self.intersections = intersections
        self.roads = roads
        
    def update(self, all_vehicles: List[Vehicle] = None):
        """
        Update vehicle with sensor readings and collision avoidance
        all_vehicles: List of all vehicles in the simulation for sensor detection
        """
        if not all_vehicles:
            all_vehicles = []
            
        # Reset speed to default unless we're yielding
        if self.current_strategy != CollisionStrategy.YIELD or self.yield_timer <= 0:
            self.speed = self.default_speed
            
        # Check if we're yielding
        if self.current_strategy == CollisionStrategy.YIELD and self.yield_timer > 0:
            # Continue yielding for a bit
            self.yield_timer -= 1
            self.speed = 0.0
            return
        
        # Sensor updates
        if hasattr(self, 'lidar') and self.lidar:
            lidar_readings = self.lidar.detect_objects(
                all_vehicles, (self.x, self.y), self.angle
            )
            self.lidar.publish_readings()
        else:
            lidar_readings = []
            
        if hasattr(self, 'radar') and self.radar:
            radar_readings = self.radar.detect_objects(
                all_vehicles, (self.x, self.y), self.angle
            )
            self.radar.publish_readings()
        else:
            radar_readings = []
            
        # Share data via IoT if available
        if hasattr(self, 'iot') and self.iot:
            # Create vehicle data
            now = self.node.get_clock().now().nanoseconds / 1e9 if self.node else 0.0
            data = VehicleData(
                vehicle_id=self.id,
                position=(self.x, self.y),
                velocity=(self.vx, self.vy),
                heading=self.angle,
                route=self.route,
                current_target=self.current_target,
                timestamp=now,
                vehicle_type=self.vehicle_type
            )
            
            # Broadcast data
            self.iot.broadcast_data(data)
            
            # Get other vehicle data
            self.other_vehicles = self.iot.known_vehicles
            
        # Process sensor data for immediate collision avoidance
        if hasattr(self, 'collision_avoidance') and self.collision_avoidance:
            collision_risk, recommended_speed = self.collision_avoidance.process_sensor_data(
                lidar_readings, radar_readings
            )
            
            # Check route conflicts if we have IoT data about other vehicles
            if self.other_vehicles and self.route and self.current_target < len(self.route):
                strategy, target_idx, conflict_speed = self.collision_avoidance.check_route_conflicts(
                    self.route, self.other_vehicles, self.current_target
                )
                
                # Apply recommended strategy
                self.current_strategy = strategy
                
                if strategy == CollisionStrategy.SLOW_DOWN:
                    # Take the more conservative speed adjustment
                    self.speed = min(recommended_speed, conflict_speed)
                    
                elif strategy == CollisionStrategy.YIELD:
                    # Stop and wait
                    self.speed = 0.0
                    self.yield_timer = 60  # Wait for ~2 seconds (assuming 30 FPS)
                    
                elif strategy == CollisionStrategy.REROUTE and not self.is_rerouting:
                    # Find alternative route
                    self.is_rerouting = True
                    if self.intersections and self.roads:
                        # Find conflict point
                        if self.current_target < len(self.route):
                            conflict_point = self.route[self.current_target]
                            # Current position as start, original goal as end
                            start = (self.x, self.y)
                            end = self.original_route[-1] if self.original_route else self.route[-1]
                            
                            # Get new route
                            new_route = self.collision_avoidance.find_alternative_route(
                                self.intersections, self.roads, start, end, conflict_point
                            )
                            
                            if new_route and len(new_route) > 1:
                                # Replace route from current position onwards
                                self.route = [start] + new_route
                                self.current_target = 1  # Start from next point
                    
                    self.is_rerouting = False
            else:
                # If no IoT data, just use sensor-based speed adjustment
                self.speed = recommended_speed
                
            # Update collision risk status for visualization
            self.collision_risk = collision_risk
        
        # Basic movement update (same as regular vehicle)
        if self.current_target < len(self.route):
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
            
            # Update velocity components based on current speed
            self.vx = math.cos(math.radians(self.angle)) * self.speed
            self.vy = math.sin(math.radians(self.angle)) * self.speed
            
            # Move car forward
            self.x += self.vx
            self.y += self.vy
            
            # Check if reached target
            distance = math.sqrt(dx*dx + dy*dy)
            if distance < 20:
                self.current_target += 1