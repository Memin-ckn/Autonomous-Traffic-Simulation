import math
import numpy as np
from enum import Enum
from typing import List, Tuple, Dict, Optional
from traffic_simulation.sensors.sensor import SensorReading
from traffic_simulation.communication.iot import VehicleData


class CollisionStrategy(Enum):
    # Available collision avoidance strategies
    SLOW_DOWN = 1
    REROUTE = 2
    YIELD = 3


class CollisionAvoidance:
    """Collision avoidance system for vehicles"""
    
    def __init__(self, vehicle_id: str, grid_size: int = 4, 
                road_width: float = 40.0, default_speed: float = 3.0):
        self.vehicle_id = vehicle_id
        self.grid_size = grid_size
        self.road_width = road_width
        self.default_speed = default_speed
        self.min_safe_distance = 50.0
        self.yield_time = 0
    
    def process_sensor_data(self, lidar_readings: List[SensorReading], 
                           radar_readings: List[SensorReading]) -> Tuple[bool, float]:
        # Analyze sensor data to detect collision risks and adjust speed
        collision_risk = False
        recommended_speed = self.default_speed
        
        # Check front-facing LIDAR readings (-30° to +30°)
        front_readings = [r for r in lidar_readings if abs(r.angle) <= 30]
        
        if front_readings:
            closest = min(front_readings, key=lambda r: r.distance)
            if closest.distance < self.min_safe_distance:
                collision_risk = True
                slow_factor = max(0.1, closest.distance / self.min_safe_distance)
                recommended_speed = self.default_speed * slow_factor
        
        # Check radar for high-speed approaching vehicles
        if radar_readings:
            for reading in radar_readings:
                if abs(reading.angle) > 45:
                    continue
                    
                if reading.velocity:
                    rel_vx, rel_vy = reading.velocity
                    rel_speed = math.sqrt(rel_vx**2 + rel_vy**2)
                    
                    if reading.distance < self.min_safe_distance * 1.5 and rel_speed > 1.0:
                        collision_risk = True
                        slow_factor = max(0.1, (reading.distance / self.min_safe_distance) * 0.8)
                        recommended_speed = min(recommended_speed, self.default_speed * slow_factor)
        
        return collision_risk, recommended_speed
    
    def check_route_conflicts(self, vehicle_route: List[Tuple[float, float]], 
                             other_vehicles: Dict[str, VehicleData], 
                             current_target: int) -> Tuple[CollisionStrategy, int, float]:
        # Check for potential conflicts with other vehicles' routes
        if not vehicle_route or current_target >= len(vehicle_route):
            return CollisionStrategy.SLOW_DOWN, current_target, self.default_speed
            
        current_pos = vehicle_route[current_target - 1] if current_target > 0 else vehicle_route[0]
        route_ahead = vehicle_route[current_target:min(len(vehicle_route), current_target + 5)]
        
        if not route_ahead:
            return CollisionStrategy.SLOW_DOWN, current_target, self.default_speed
        
        # Analyze each vehicle for route intersections
        for vehicle_id, data in other_vehicles.items():
            if not data.route or data.current_target >= len(data.route):
                continue
                
            other_pos = data.position
            other_route = data.route[data.current_target:min(len(data.route), data.current_target + 5)]
            
            for i, our_point in enumerate(route_ahead):
                for j, their_point in enumerate(other_route):
                    dx = our_point[0] - their_point[0]
                    dy = our_point[1] - their_point[1]
                    dist = math.sqrt(dx*dx + dy*dy)
                    
                    if dist < self.road_width:
                        # Calculate arrival times to intersection
                        our_distance = self._calculate_route_distance(vehicle_route, current_target, current_target + i)
                        their_distance = self._calculate_route_distance(data.route, data.current_target, data.current_target + j)
                        
                        our_time = our_distance / self.default_speed
                        their_time = their_distance / data.velocity[0] if data.velocity[0] > 0 else 999999
                        time_diff = abs(our_time - their_time)
                        
                        # Handle potential collision based on timing
                        if time_diff < 2.0:
                            if data.vehicle_type == "emergency": #(not implemented yet)
                                return CollisionStrategy.YIELD, current_target, 0.0
                            
                            if vehicle_id < self.vehicle_id:
                                if our_time < their_time:
                                    return CollisionStrategy.SLOW_DOWN, current_target, self.default_speed * 1.2
                                else:
                                    return CollisionStrategy.SLOW_DOWN, current_target, self.default_speed * 0.5
                            else:
                                return CollisionStrategy.SLOW_DOWN, current_target, self.default_speed * 0.9
                                
                        if i == 0 and our_time > 3.0:
                            return CollisionStrategy.REROUTE, current_target, self.default_speed
        
        return CollisionStrategy.SLOW_DOWN, current_target, self.default_speed
    
    def _calculate_route_distance(self, route: List[Tuple[float, float]], 
                                 start_idx: int, end_idx: int) -> float:
        # Calculate total distance between route points
        distance = 0.0
        for i in range(start_idx, end_idx):
            if i + 1 < len(route):
                dx = route[i+1][0] - route[i][0]
                dy = route[i+1][1] - route[i][1]
                distance += math.sqrt(dx*dx + dy*dy)
        return distance
    
    def find_alternative_route(self, intersections: List[Tuple[float, float]],
                              roads: List[Tuple[Tuple[float, float], Tuple[float, float]]],
                              start: Tuple[float, float], end: Tuple[float, float],
                              conflict_point: Tuple[float, float]) -> List[Tuple[float, float]]:
        # Find alternative path avoiding conflict using A* with increased cost near conflict
        if not intersections or not roads:
            return [start, end]
            
        # Build road network graph
        adjacency = {i: [] for i in intersections}
        for road in roads:
            a, b = road
            if a in adjacency:
                adjacency[a].append(b)
            if b in adjacency:
                adjacency[b].append(a)
        
        start_node = self._find_nearest_intersection(start, intersections)
        end_node = self._find_nearest_intersection(end, intersections)
        
        return self._a_star_search(adjacency, start_node, end_node, conflict_point)
    
    def _find_nearest_intersection(self, point: Tuple[float, float], 
                                 intersections: List[Tuple[float, float]]) -> Tuple[float, float]:
        # Find closest intersection to given point
        closest_dist = float('inf')
        closest_point = None
        
        for intersection in intersections:
            dx = point[0] - intersection[0]
            dy = point[1] - intersection[1]
            dist = math.sqrt(dx*dx + dy*dy)
            
            if dist < closest_dist:
                closest_dist = dist
                closest_point = intersection
                
        return closest_point
    
    def _a_star_search(self, adjacency: Dict[Tuple[float, float], List[Tuple[float, float]]],
                     start: Tuple[float, float], goal: Tuple[float, float], 
                     avoid_point: Tuple[float, float]) -> List[Tuple[float, float]]:
        # A* pathfinding with penalty for proximity to avoid_point
        def heuristic(node, goal):
            dx = node[0] - goal[0]
            dy = node[1] - goal[1]
            return math.sqrt(dx*dx + dy*dy)
        
        def edge_cost(a, b, avoid):
            # Base cost plus penalty for proximity to avoid_point
            dx = b[0] - a[0]
            dy = b[1] - a[1]
            cost = math.sqrt(dx*dx + dy*dy)
            
            if avoid:
                dx_avoid = b[0] - avoid[0]
                dy_avoid = b[1] - avoid[1]
                dist_to_avoid = math.sqrt(dx_avoid*dx_avoid + dy_avoid*dy_avoid)
                penalty = 100.0 / max(1.0, dist_to_avoid)
                cost += penalty
                
            return cost
        
        # A* algorithm
        open_set = {start}
        closed_set = set()
        
        # Costs and parents
        g_score = {start: 0}
        f_score = {start: heuristic(start, goal)}
        came_from = {}
        
        while open_set:
            # Find node with lowest f_score
            current = min(open_set, key=lambda node: f_score.get(node, float('inf')))
            
            # Check if we reached the goal
            if current == goal:
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return list(reversed(path))
                
            # Move current from open to closed
            open_set.remove(current)
            closed_set.add(current)
            
            # Check neighbors
            for neighbor in adjacency.get(current, []):
                if neighbor in closed_set:
                    continue
                    
                # Calculate tentative g_score
                tentative_g = g_score[current] + edge_cost(current, neighbor, avoid_point)
                
                if neighbor not in open_set:
                    open_set.add(neighbor)
                elif tentative_g >= g_score.get(neighbor, float('inf')):
                    continue
                    
                # This path is the best so far
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
        
        # No path found
        return [start, goal]  # Fallback to direct route 