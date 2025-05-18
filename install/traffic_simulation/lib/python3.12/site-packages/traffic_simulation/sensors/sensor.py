import rclpy
import math
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional
from sensor_msgs.msg import LaserScan, Range
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

@dataclass
class SensorReading:
    # Common data structure for all sensor readings
    distance: float      # Distance to detected object
    angle: float        # Angle relative to sensor heading
    object_id: Optional[str] = None  # Unique identifier of detected object
    velocity: Optional[Tuple[float, float]] = None  # Velocity vector (vx, vy)
    timestamp: float = 0.0  # Time of reading


class Sensor:
    # Base sensor class defining common interface
    def __init__(self, parent_id: str, range_max: float = 100.0, angle_range: float = 180.0, 
                 angle_increment: float = 1.0, node=None):
        self.parent_id = parent_id
        self.range_max = range_max
        self.angle_range = angle_range  # Field of view in degrees
        self.angle_increment = angle_increment  # Angular resolution
        self.node = node
        self.readings = []
        
    def detect_objects(self, objects: List, position: Tuple[float, float], 
                      heading: float) -> List[SensorReading]:
        pass
    
    def publish_readings(self):
        pass


class Lidar(Sensor):
    # LIDAR implementation with high-resolution 360° scanning
    def __init__(self, parent_id: str, range_max: float = 150.0, 
                 angle_range: float = 360.0, angle_increment: float = 1.0, node=None):
        super().__init__(parent_id, range_max, angle_range, angle_increment, node)
        
        # Setup ROS2 publisher with best-effort QoS for high-frequency data
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
        # Perform 360° scan and detect objects
        self.readings = []
        
        # Generate scan angles
        start_angle = heading - self.angle_range / 2
        angles = [start_angle + i * self.angle_increment 
                  for i in range(int(self.angle_range / self.angle_increment) + 1)]
        
        # Ray-cast at each angle
        for angle in angles:
            rad_angle = math.radians(angle)
            closest_dist = self.range_max
            closest_obj = None
            
            # Find closest object at current angle
            for obj in objects:
                if hasattr(obj, 'id') and obj.id == self.parent_id:
                    continue
                
                if hasattr(obj, 'x') and hasattr(obj, 'y'):
                    obj_x, obj_y = obj.x, obj.y
                    dx = obj_x - position[0]
                    dy = obj_y - position[1]
                    center_dist = math.sqrt(dx*dx + dy*dy)
                    
                    if center_dist < self.range_max:
                        obj_angle = math.degrees(math.atan2(dy, dx))
                        angle_diff = abs((obj_angle - angle + 180) % 360 - 180)
                        
                        # Object within 5° beam width
                        if angle_diff < 5 and center_dist < closest_dist:
                            closest_dist = center_dist
                            closest_obj = obj
            
            # Record detection
            if closest_obj is not None:
                reading = SensorReading(
                    distance=closest_dist,
                    angle=angle - heading,
                    object_id=getattr(closest_obj, 'id', None),
                    velocity=(getattr(closest_obj, 'vx', 0), getattr(closest_obj, 'vy', 0)),
                    timestamp=self.node.get_clock().now().nanoseconds / 1e9 if self.node else 0.0
                )
                self.readings.append(reading)
            else:
                # No detection at this angle
                reading = SensorReading(
                    distance=self.range_max,
                    angle=angle - heading,
                    timestamp=self.node.get_clock().now().nanoseconds / 1e9 if self.node else 0.0
                )
                self.readings.append(reading)
        
        return self.readings
    
    def publish_readings(self):
        # Publish scan data as ROS2 LaserScan message
        if not self.node or not hasattr(self, 'publisher'):
            return
            
        msg = LaserScan()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = f"{self.parent_id}_lidar"
        
        msg.angle_min = math.radians(-self.angle_range / 2)
        msg.angle_max = math.radians(self.angle_range / 2)
        msg.angle_increment = math.radians(self.angle_increment)
        msg.time_increment = 0.0
        msg.scan_time = 0.1  # 10Hz scan rate
        msg.range_min = 0.1
        msg.range_max = self.range_max
        
        msg.ranges = [reading.distance for reading in self.readings]
        self.publisher.publish(msg)


class Radar(Sensor):
    # Radar implementation with wider beams and velocity detection
    def __init__(self, parent_id: str, range_max: float = 200.0, 
                 angle_range: float = 120.0, angle_increment: float = 5.0, node=None):
        super().__init__(parent_id, range_max, angle_range, angle_increment, node)
        
        # Setup ROS2 publisher for radar data
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
        # Detect objects with velocity information
        self.readings = []
        
        # Generate scan angles (wider spacing than LIDAR)
        start_angle = heading - self.angle_range / 2
        angles = [start_angle + i * self.angle_increment 
                  for i in range(int(self.angle_range / self.angle_increment) + 1)]
        
        # Ray-cast with wider beams
        for angle in angles:
            rad_angle = math.radians(angle)
            closest_dist = self.range_max
            closest_obj = None
            
            for obj in objects:
                if hasattr(obj, 'id') and obj.id == self.parent_id:
                    continue
                
                if hasattr(obj, 'x') and hasattr(obj, 'y'):
                    obj_x, obj_y = obj.x, obj.y
                    dx = obj_x - position[0]
                    dy = obj_y - position[1]
                    center_dist = math.sqrt(dx*dx + dy*dy)
                    
                    if center_dist < self.range_max:
                        obj_angle = math.degrees(math.atan2(dy, dx))
                        angle_diff = abs((obj_angle - angle + 180) % 360 - 180)
                        
                        # Object within 10° beam width (wider than LIDAR)
                        if angle_diff < 10 and center_dist < closest_dist:
                            closest_dist = center_dist
                            closest_obj = obj
            
            # Record detection with velocity
            if closest_obj is not None:
                vx = getattr(closest_obj, 'vx', 0)
                vy = getattr(closest_obj, 'vy', 0)
                
                reading = SensorReading(
                    distance=closest_dist,
                    angle=angle - heading,
                    object_id=getattr(closest_obj, 'id', None),
                    velocity=(vx, vy),
                    timestamp=self.node.get_clock().now().nanoseconds / 1e9 if self.node else 0.0
                )
                self.readings.append(reading)
        
        return self.readings
    
    def publish_readings(self):
        # Publish closest detection as ROS2 Range message
        if not self.node or not hasattr(self, 'publisher') or not self.readings:
            return
            
        closest_reading = min(self.readings, key=lambda r: r.distance)
        
        msg = Range()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = f"{self.parent_id}_radar"
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = math.radians(self.angle_range)
        msg.min_range = 0.1
        msg.max_range = self.range_max
        msg.range = closest_reading.distance
        
        self.publisher.publish(msg) 