import rclpy
import math
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional
from sensor_msgs.msg import LaserScan, Range
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

@dataclass
class SensorReading:
    """Data class to hold sensor readings"""
    distance: float
    angle: float
    object_id: Optional[str] = None  # ID of detected object
    velocity: Optional[Tuple[float, float]] = None  # Optional velocity vector of detected object
    timestamp: float = 0.0  # When reading was taken


class Sensor:
    """Base class for all sensors"""
    def __init__(self, parent_id: str, range_max: float = 100.0, angle_range: float = 180.0, 
                 angle_increment: float = 1.0, node=None):
        self.parent_id = parent_id
        self.range_max = range_max
        self.angle_range = angle_range  # in degrees
        self.angle_increment = angle_increment  # in degrees
        self.node = node
        self.readings = []
        
    def detect_objects(self, objects: List, position: Tuple[float, float], 
                      heading: float) -> List[SensorReading]:
        """Detect objects within sensor range"""
        pass
    
    def publish_readings(self):
        """Publish sensor readings to ROS2 topics"""
        pass


class Lidar(Sensor):
    """Lidar sensor with 360 degree view"""
    def __init__(self, parent_id: str, range_max: float = 150.0, 
                 angle_range: float = 360.0, angle_increment: float = 1.0, node=None):
        super().__init__(parent_id, range_max, angle_range, angle_increment, node)
        
        # Create ROS2 publisher if node is available
        if self.node:
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
            self.publisher = self.node.create_publisher(
                LaserScan, 
                f'/{parent_id}/lidar/scan',
                qos
            )
        
    def detect_objects(self, objects: List, position: Tuple[float, float], 
                      heading: float) -> List[SensorReading]:
        """
        Detect objects within the lidar range
        position: (x, y) of the sensor
        heading: Orientation in degrees
        """
        # Clear previous readings
        self.readings = []
        
        # Calculate angles to check (relative to vehicle heading)
        start_angle = heading - self.angle_range / 2
        angles = [start_angle + i * self.angle_increment 
                  for i in range(int(self.angle_range / self.angle_increment) + 1)]
        
        # For each angle, find closest object
        for angle in angles:
            # Convert to radians
            rad_angle = math.radians(angle)
            
            # Calculate ray end point at maximum range
            ray_end_x = position[0] + self.range_max * math.cos(rad_angle)
            ray_end_y = position[1] + self.range_max * math.sin(rad_angle)
            
            closest_dist = self.range_max
            closest_obj = None
            
            # Check all objects
            for obj in objects:
                # Skip self
                if hasattr(obj, 'id') and obj.id == self.parent_id:
                    continue
                
                # Simple circle-line intersection for detection
                # This is a simplification - in a real system we would use proper collision detection
                if hasattr(obj, 'x') and hasattr(obj, 'y'):
                    obj_x, obj_y = obj.x, obj.y
                    
                    # Calculate distance to object center
                    dx = obj_x - position[0]
                    dy = obj_y - position[1]
                    center_dist = math.sqrt(dx*dx + dy*dy)
                    
                    # If within range
                    if center_dist < self.range_max:
                        # Calculate angle to object
                        obj_angle = math.degrees(math.atan2(dy, dx))
                        angle_diff = abs((obj_angle - angle + 180) % 360 - 180)
                        
                        # If within beam width (10 degrees)
                        if angle_diff < 5:
                            # Check if this is the closest object
                            if center_dist < closest_dist:
                                closest_dist = center_dist
                                closest_obj = obj
            
            # Add reading
            if closest_obj is not None:
                reading = SensorReading(
                    distance=closest_dist,
                    angle=angle - heading,  # Relative to vehicle heading
                    object_id=getattr(closest_obj, 'id', None),
                    velocity=(getattr(closest_obj, 'vx', 0), getattr(closest_obj, 'vy', 0)),
                    timestamp=self.node.get_clock().now().nanoseconds / 1e9 if self.node else 0.0
                )
                self.readings.append(reading)
            else:
                # No object detected at this angle
                reading = SensorReading(
                    distance=self.range_max,
                    angle=angle - heading,
                    timestamp=self.node.get_clock().now().nanoseconds / 1e9 if self.node else 0.0
                )
                self.readings.append(reading)
        
        return self.readings
    
    def publish_readings(self):
        """Publish lidar readings to ROS2 topic"""
        if not self.node or not hasattr(self, 'publisher'):
            return
            
        # Create LaserScan message
        msg = LaserScan()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = f"{self.parent_id}_lidar"
        
        # Set scan parameters
        msg.angle_min = math.radians(-self.angle_range / 2)
        msg.angle_max = math.radians(self.angle_range / 2)
        msg.angle_increment = math.radians(self.angle_increment)
        msg.time_increment = 0.0
        msg.scan_time = 0.1  # 10Hz scan rate
        msg.range_min = 0.1
        msg.range_max = self.range_max
        
        # Set ranges from readings
        ranges = []
        for reading in self.readings:
            ranges.append(reading.distance)
        
        msg.ranges = ranges
        
        # Publish
        self.publisher.publish(msg)


class Radar(Sensor):
    """Radar sensor with velocity detection capabilities"""
    def __init__(self, parent_id: str, range_max: float = 200.0, 
                 angle_range: float = 120.0, angle_increment: float = 5.0, node=None):
        super().__init__(parent_id, range_max, angle_range, angle_increment, node)
        
        # Create ROS2 publisher if node is available
        if self.node:
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
            self.publisher = self.node.create_publisher(
                Range, 
                f'/{parent_id}/radar/range',
                qos
            )
    
    def detect_objects(self, objects: List, position: Tuple[float, float], 
                      heading: float) -> List[SensorReading]:
        """
        Detect objects within radar range with velocity information
        position: (x, y) of the sensor
        heading: Orientation in degrees
        """
        # Similar to Lidar but with fewer beams and velocity calculation
        # Clear previous readings
        self.readings = []
        
        # Calculate angles to check (relative to vehicle heading)
        start_angle = heading - self.angle_range / 2
        angles = [start_angle + i * self.angle_increment 
                  for i in range(int(self.angle_range / self.angle_increment) + 1)]
        
        # For each angle, find objects
        for angle in angles:
            # Convert to radians
            rad_angle = math.radians(angle)
            
            # Calculate ray end point at maximum range
            ray_end_x = position[0] + self.range_max * math.cos(rad_angle)
            ray_end_y = position[1] + self.range_max * math.sin(rad_angle)
            
            closest_dist = self.range_max
            closest_obj = None
            
            # Check all objects
            for obj in objects:
                # Skip self
                if hasattr(obj, 'id') and obj.id == self.parent_id:
                    continue
                
                # Simple circle-line intersection for detection
                if hasattr(obj, 'x') and hasattr(obj, 'y'):
                    obj_x, obj_y = obj.x, obj.y
                    
                    # Calculate distance to object center
                    dx = obj_x - position[0]
                    dy = obj_y - position[1]
                    center_dist = math.sqrt(dx*dx + dy*dy)
                    
                    # If within range
                    if center_dist < self.range_max:
                        # Calculate angle to object
                        obj_angle = math.degrees(math.atan2(dy, dx))
                        angle_diff = abs((obj_angle - angle + 180) % 360 - 180)
                        
                        # If within beam width (wider for radar)
                        if angle_diff < 10:
                            # Check if this is the closest object
                            if center_dist < closest_dist:
                                closest_dist = center_dist
                                closest_obj = obj
            
            # Add reading
            if closest_obj is not None:
                # Calculate relative velocity (if available)
                vx = getattr(closest_obj, 'vx', 0)
                vy = getattr(closest_obj, 'vy', 0)
                
                reading = SensorReading(
                    distance=closest_dist,
                    angle=angle - heading,  # Relative to vehicle heading
                    object_id=getattr(closest_obj, 'id', None),
                    velocity=(vx, vy),
                    timestamp=self.node.get_clock().now().nanoseconds / 1e9 if self.node else 0.0
                )
                self.readings.append(reading)
        
        return self.readings
    
    def publish_readings(self):
        """Publish radar readings to ROS2 topic"""
        if not self.node or not hasattr(self, 'publisher') or not self.readings:
            return
            
        # Find closest object for Range message
        closest_reading = min(self.readings, key=lambda r: r.distance)
        
        # Create Range message
        msg = Range()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = f"{self.parent_id}_radar"
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = math.radians(self.angle_range)
        msg.min_range = 0.1
        msg.max_range = self.range_max
        msg.range = closest_reading.distance
        
        # Publish
        self.publisher.publish(msg) 