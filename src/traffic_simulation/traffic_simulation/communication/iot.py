import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose, PoseStamped
from dataclasses import dataclass
from typing import List, Dict, Tuple, Any, Optional
import json
import uuid

@dataclass
class VehicleData:
    """Data shared between vehicles via IoT"""
    vehicle_id: str
    position: Tuple[float, float]
    velocity: Tuple[float, float]
    heading: float
    route: List[Tuple[float, float]]
    current_target: int
    timestamp: float
    vehicle_type: str = "regular"  # "regular" or "emergency"
    status: Dict[str, Any] = None  # Custom status information


class IoTCommunicator:
    """Handles vehicle-to-vehicle (V2V) and vehicle-to-infrastructure (V2I) communication"""
    def __init__(self, vehicle_id: str, node: Node):
        self.vehicle_id = vehicle_id
        self.node = node
        self.known_vehicles: Dict[str, VehicleData] = {}
        
        # Initialize publishers and subscribers if node provided
        if self.node:
            # Publisher for vehicle data
            self.data_publisher = self.node.create_publisher(
                String,
                '/traffic/vehicle_data',
                10
            )
            
            # Subscriber for other vehicle data
            self.data_subscriber = self.node.create_subscription(
                String,
                '/traffic/vehicle_data',
                self.data_callback,
                10
            )
    
    def broadcast_data(self, data: VehicleData):
        """Broadcast vehicle data to the network"""
        if not self.node or not hasattr(self, 'data_publisher'):
            return
            
        # Convert to dictionary
        data_dict = {
            'vehicle_id': data.vehicle_id,
            'position': data.position,
            'velocity': data.velocity,
            'heading': data.heading,
            'route': data.route,
            'current_target': data.current_target,
            'timestamp': data.timestamp,
            'vehicle_type': data.vehicle_type
        }
        
        if data.status:
            data_dict['status'] = data.status
        
        # Convert to JSON
        json_data = json.dumps(data_dict)
        
        # Create and publish message
        msg = String()
        msg.data = json_data
        self.data_publisher.publish(msg)
    
    def data_callback(self, msg: String):
        """Process incoming vehicle data"""
        try:
            # Parse JSON data
            data_dict = json.loads(msg.data)
            
            # Skip own messages
            if data_dict['vehicle_id'] == self.vehicle_id:
                return
                
            # Create VehicleData object
            vehicle_data = VehicleData(
                vehicle_id=data_dict['vehicle_id'],
                position=tuple(data_dict['position']),
                velocity=tuple(data_dict['velocity']),
                heading=data_dict['heading'],
                route=[tuple(p) for p in data_dict['route']],
                current_target=data_dict['current_target'],
                timestamp=data_dict['timestamp'],
                vehicle_type=data_dict.get('vehicle_type', 'regular')
            )
            
            if 'status' in data_dict:
                vehicle_data.status = data_dict['status']
            
            # Update known vehicles dictionary
            self.known_vehicles[vehicle_data.vehicle_id] = vehicle_data
            
        except (json.JSONDecodeError, KeyError) as e:
            if self.node:
                self.node.get_logger().error(f"Error processing vehicle data: {str(e)}")
    
    def get_nearby_vehicles(self, max_distance: float = 200.0) -> List[VehicleData]:
        """Get list of vehicles within specified distance"""
        if not hasattr(self, 'position'):
            return []
            
        nearby = []
        for vehicle_id, data in self.known_vehicles.items():
            # Calculate distance
            dx = data.position[0] - self.position[0]
            dy = data.position[1] - self.position[1]
            distance = (dx*dx + dy*dy) ** 0.5
            
            if distance <= max_distance:
                nearby.append(data)
                
        return nearby
    
    def get_route_conflicts(self, vehicle_route: List[Tuple[float, float]], 
                           horizon: int = 5) -> List[Tuple[str, int, Tuple[float, float]]]:
        """
        Find potential route conflicts with other vehicles
        
        Returns list of tuples containing:
        - vehicle_id: ID of vehicle with potential conflict
        - segment_index: Index in the route where conflict occurs
        - position: (x, y) of the conflict point
        """
        conflicts = []
        
        # Only check up to horizon steps in the route
        check_route = vehicle_route[:min(len(vehicle_route), horizon)]
        
        for vehicle_id, data in self.known_vehicles.items():
            # Skip if route is empty or invalid
            if not data.route or data.current_target >= len(data.route):
                continue
            
            # Get other vehicle's upcoming route
            other_route = data.route[data.current_target:min(len(data.route), 
                                                         data.current_target + horizon)]
            
            # Check for intersections between routes
            for i, point1 in enumerate(check_route[:-1]):
                for other_point in other_route:
                    # Simple distance check for potential conflict
                    dx = point1[0] - other_point[0]
                    dy = point1[1] - other_point[1]
                    distance = (dx*dx + dy*dy) ** 0.5
                    
                    # If points are close, consider it a potential conflict
                    if distance < 30:  # Threshold for conflict detection
                        conflicts.append((vehicle_id, i, point1))
                        break
        
        return conflicts 