import math
import numpy as np
from enum import Enum
from typing import List, Tuple, Dict, Optional
from traffic_simulation.sensors.sensor import SensorReading
from traffic_simulation.communication.iot import VehicleData


class CollisionStrategy(Enum):
    """Different collision avoidance strategies"""
    SLOW_DOWN = 1  # Reduce speed to let other vehicle pass
    REROUTE = 2    # Find an alternative route
    YIELD = 3      # Stop and wait for conflict to clear


class CollisionAvoidance:
    """Collision avoidance system for vehicles"""
    
    def __init__(self, vehicle_id: str, grid_size: int = 4, 
                road_width: float = 40.0, default_speed: float = 3.0):
        self.vehicle_id = vehicle_id
        self.grid_size = grid_size
        self.road_width = road_width
        self.default_speed = default_speed
        self.min_safe_distance = 50.0  # Minimum safe distance to keep from other vehicles
        self.yield_time = 0  # Counter for how long to yield
    
    def process_sensor_data(self, lidar_readings: List[SensorReading], 
                           radar_readings: List[SensorReading]) -> Tuple[bool, float]:
        """
        Process sensor readings to determine if collision avoidance is needed
        Returns:
            - collision_risk: bool indicating if there's risk of collision
            - recommended_speed: float with recommended speed adjustment
        """
        collision_risk = False
        recommended_speed = self.default_speed
        
        # Find closest object in front of vehicle (roughly -30 to +30 degrees)
        front_readings = [r for r in lidar_readings if abs(r.angle) <= 30]
        
        if front_readings:
            # Find closest reading
            closest = min(front_readings, key=lambda r: r.distance)
            
            # If object is too close, reduce speed based on distance
            if closest.distance < self.min_safe_distance:
                collision_risk = True
                
                # Calculate speed reduction factor based on distance
                # The closer the object, the more we slow down
                slow_factor = max(0.1, closest.distance / self.min_safe_distance)
                recommended_speed = self.default_speed * slow_factor
        
        # Check radar for high-speed approaching vehicles
        if radar_readings:
            for reading in radar_readings:
                # Skip objects not directly ahead or behind
                if abs(reading.angle) > 45:
                    continue
                    
                # If we have velocity data
                if reading.velocity:
                    # Calculate relative velocity
                    rel_vx, rel_vy = reading.velocity
                    # Convert to relative speed towards/away from us
                    rel_speed = math.sqrt(rel_vx**2 + rel_vy**2)
                    
                    # If object is approaching fast and close
                    if reading.distance < self.min_safe_distance * 1.5 and rel_speed > 1.0:
                        collision_risk = True
                        # Slow down more for faster vehicles
                        slow_factor = max(0.1, (reading.distance / self.min_safe_distance) * 0.8)
                        recommended_speed = min(recommended_speed, self.default_speed * slow_factor)
        
        return collision_risk, recommended_speed
    
    def check_route_conflicts(self, vehicle_route: List[Tuple[float, float]], 
                             other_vehicles: Dict[str, VehicleData], 
                             current_target: int) -> Tuple[CollisionStrategy, int, float]:
        """
        Check for potential route conflicts using IoT data from other vehicles
        
        Returns:
            - strategy: CollisionStrategy to apply
            - target_idx: Index in route to reroute from (if applicable)
            - speed: Recommended speed adjustment
        """
        if not vehicle_route or current_target >= len(vehicle_route):
            return CollisionStrategy.SLOW_DOWN, current_target, self.default_speed
            
        # Get our current position and next few waypoints
        current_pos = vehicle_route[current_target - 1] if current_target > 0 else vehicle_route[0]
        route_ahead = vehicle_route[current_target:min(len(vehicle_route), current_target + 5)]
        
        # No future route to check
        if not route_ahead:
            return CollisionStrategy.SLOW_DOWN, current_target, self.default_speed
        
        # Check each vehicle for potential conflicts
        for vehicle_id, data in other_vehicles.items():
            # Skip vehicles without route data
            if not data.route or data.current_target >= len(data.route):
                continue
                
            # Get other vehicle's current position and future route
            other_pos = data.position
            other_route = data.route[data.current_target:min(len(data.route), data.current_target + 5)]
            
            # Check if routes intersect
            for i, our_point in enumerate(route_ahead):
                for j, their_point in enumerate(other_route):
                    # Calculate distance between waypoints
                    dx = our_point[0] - their_point[0]
                    dy = our_point[1] - their_point[1]
                    dist = math.sqrt(dx*dx + dy*dy)
                    
                    # If waypoints are close (potential conflict)
                    if dist < self.road_width:
                        # Calculate time-to-intersection for both vehicles
                        # This is a simplified calculation
                        our_distance = self._calculate_route_distance(vehicle_route, current_target, current_target + i)
                        their_distance = self._calculate_route_distance(data.route, data.current_target, data.current_target + j)
                        
                        # Assuming constant speed
                        our_time = our_distance / self.default_speed
                        their_time = their_distance / data.velocity[0] if data.velocity[0] > 0 else 999999
                        
                        # Time difference - how close our arrivals are
                        time_diff = abs(our_time - their_time)
                        
                        # If we'll arrive at almost the same time (within 2 seconds)
                        if time_diff < 2.0:
                            # If other vehicle is emergency vehicle, always yield
                            if data.vehicle_type == "emergency":
                                return CollisionStrategy.YIELD, current_target, 0.0  # Full stop
                            
                            # If our ID is "lower" (arbitrary priority), we yield
                            if vehicle_id < self.vehicle_id:
                                if our_time < their_time:
                                    # We'll arrive first, so speed up slightly to clear intersection
                                    return CollisionStrategy.SLOW_DOWN, current_target, self.default_speed * 1.2
                                else:
                                    # We'll arrive after, so slow down to let them through
                                    return CollisionStrategy.SLOW_DOWN, current_target, self.default_speed * 0.5
                            else:
                                # We have priority - maintain speed but be cautious
                                return CollisionStrategy.SLOW_DOWN, current_target, self.default_speed * 0.9
                                
                        # If they'll arrive well before us, maintain speed
                        elif their_time < our_time - 2.0:
                            pass
                            
                        # If we'll arrive well before them, maintain speed
                        elif our_time < their_time - 2.0:
                            pass
                            
                        # If this is a serious future conflict, consider rerouting
                        if i == 0 and our_time > 3.0:  # If conflict is not immediate
                            return CollisionStrategy.REROUTE, current_target, self.default_speed
        
        # No conflicts found
        return CollisionStrategy.SLOW_DOWN, current_target, self.default_speed
    
    def _calculate_route_distance(self, route: List[Tuple[float, float]], 
                                 start_idx: int, end_idx: int) -> float:
        """Calculate distance along a route between two indices"""
        distance = 0.0
        for i in range(start_idx, end_idx):
            if i + 1 < len(route):
                # Add segment length
                dx = route[i+1][0] - route[i][0]
                dy = route[i+1][1] - route[i][1]
                distance += math.sqrt(dx*dx + dy*dy)
        return distance
    
    def find_alternative_route(self, intersections: List[Tuple[float, float]],
                              roads: List[Tuple[Tuple[float, float], Tuple[float, float]]],
                              start: Tuple[float, float], end: Tuple[float, float],
                              conflict_point: Tuple[float, float]) -> List[Tuple[float, float]]:
        """
        Find an alternative route avoiding a known conflict point
        Uses simple A* with increased cost near conflict point
        """
        if not intersections or not roads:
            return [start, end]
            
        # Build adjacency list for intersections
        adjacency = {i: [] for i in intersections}
        for road in roads:
            a, b = road
            # Road connects points a and b in both directions
            if a in adjacency:
                adjacency[a].append(b)
            if b in adjacency:
                adjacency[b].append(a)
        
        # Find nearest intersection to start and end
        start_node = self._find_nearest_intersection(start, intersections)
        end_node = self._find_nearest_intersection(end, intersections)
        
        # A* search for best path
        return self._a_star_search(adjacency, start_node, end_node, conflict_point)
    
    def _find_nearest_intersection(self, point: Tuple[float, float], 
                                 intersections: List[Tuple[float, float]]) -> Tuple[float, float]:
        """Find the nearest intersection to the given point"""
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
        """A* search algorithm to find optimal path avoiding a conflict point"""
        # Define heuristic (straight-line distance)
        def heuristic(node, goal):
            dx = node[0] - goal[0]
            dy = node[1] - goal[1]
            return math.sqrt(dx*dx + dy*dy)
        
        # Define edge cost with penalty for proximity to avoid_point
        def edge_cost(a, b, avoid):
            # Basic cost is distance
            dx = b[0] - a[0]
            dy = b[1] - a[1]
            cost = math.sqrt(dx*dx + dy*dy)
            
            # Add penalty based on proximity to avoid_point
            if avoid:
                dx_avoid = b[0] - avoid[0]
                dy_avoid = b[1] - avoid[1]
                dist_to_avoid = math.sqrt(dx_avoid*dx_avoid + dy_avoid*dy_avoid)
                
                # Penalty increases as distance to avoid point decreases
                # 100 is a large penalty for being very close
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