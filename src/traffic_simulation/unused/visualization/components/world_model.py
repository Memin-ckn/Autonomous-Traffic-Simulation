#!/usr/bin/env python3

"""
World model component for the traffic simulation.
Manages the road network, intersections, and map generation.
"""

import random
import math


class WorldModel:
    """
    Manages the road network, intersections, and map generation.
    Provides functions for pathfinding and route validation.
    """
    def __init__(self, width, height, grid_size=4):
        """
        Initialize the world model with given screen dimensions
        
        Args:
            width (int): Screen width
            height (int): Screen height
            grid_size (int): Initial size of the road grid
        """
        self.width = width
        self.height = height
        self.grid_size = grid_size
        
        # Initialize intersections and roads
        self.intersections = []
        self.roads = []
        
        # Generate the initial grid
        self.generate_grid()
    
    def generate_grid(self):
        """
        Generate a grid of intersections and roads
        Returns:
            tuple: (intersections, roads) lists
        """
        grid_size = self.grid_size  # Use the current grid size
        intersections = []
        roads = []
        
        # Create a grid of intersection points
        cell_width = self.width // (grid_size + 1)
        cell_height = self.height // (grid_size + 1)
        
        for i in range(grid_size):
            for j in range(grid_size):
                x = (j + 1) * cell_width
                y = (i + 1) * cell_height
                intersections.append((x, y))
        
        # Connect adjacent intersections with roads
        for i in range(len(intersections)):
            for j in range(i+1, len(intersections)):
                x1, y1 = intersections[i]
                x2, y2 = intersections[j]
                
                # Only connect if they're adjacent (within certain distance)
                dx = abs(x1 - x2)
                dy = abs(y1 - y2)
                
                # Add some randomness to road placement
                if (dx <= cell_width * 1.5 and dy <= cell_height * 1.5) and random.random() < 0.7:
                    # Store as tuple of tuples to ensure proper lookup
                    road = (tuple(intersections[i]), tuple(intersections[j]))
                    roads.append(road)
        
        # Update class variables
        self.intersections = intersections
        self.roads = roads
        
        # Debug output
        print(f"Generated grid with {len(intersections)} intersections and {len(roads)} roads")
        return intersections, roads
    
    def update_grid_size(self, new_size):
        """
        Update the grid size and regenerate the grid
        
        Args:
            new_size (int): New grid size
        
        Returns:
            tuple: (intersections, roads) lists
        """
        self.grid_size = new_size
        return self.generate_grid()
    
    def has_road_between(self, point1, point2):
        """
        Check if there's a direct road between two points
        
        Args:
            point1 (tuple): First point (x, y)
            point2 (tuple): Second point (x, y)
            
        Returns:
            bool: True if a road exists, False otherwise
        """
        return (point1, point2) in self.roads or (point2, point1) in self.roads
    
    def find_path(self, start, end):
        """
        Find a path from start to end using only existing roads
        Uses breadth-first search
        
        Args:
            start (tuple): Start point (x, y)
            end (tuple): End point (x, y)
            
        Returns:
            list: List of points forming the path, or None if no path exists
        """
        if start == end:
            return [start]
            
        # Check if there's a direct road between start and end
        if self.has_road_between(start, end):
            return [start, end]
        
        # Build an adjacency list based on actual roads
        adjacency = {}
        for point in self.intersections:
            adjacency[point] = []
            
        # Add connected points based on actual roads
        for road in self.roads:
            a, b = road
            # Add both directions
            if a in adjacency:
                adjacency[a].append(b)
            if b in adjacency:
                adjacency[b].append(a)
        
        # Check if either start or end is not in adjacency list
        if start not in adjacency or end not in adjacency:
            return None  # No path possible
        
        # Breadth-first search
        visited = set()
        queue = [(start, [start])]
        
        while queue:
            current, path = queue.pop(0)
            
            if current == end:
                # Verify each segment is a valid road
                for i in range(len(path)-1):
                    if not self.has_road_between(path[i], path[i+1]):
                        print(f"Invalid segment: {path[i]} to {path[i+1]}")
                        return None  # Invalid path segment
                return path
                
            if current in visited:
                continue
                
            visited.add(current)
            
            # Add neighbors from adjacency list to ensure only existing roads are used
            for neighbor in adjacency.get(current, []):
                if neighbor not in visited:
                    queue.append((neighbor, path + [neighbor]))
        
        return None  # No path found
    
    def find_path_avoiding_ghosts(self, start, end, avoid_node=None, ghost_positions=None):
        """
        Find a path that avoids obstacles (like Pacman avoiding ghosts)
        Uses A* search algorithm with penalties for paths near obstacles
        
        Args:
            start (tuple): Start point (x, y)
            end (tuple): End point (x, y)
            avoid_node (tuple): Specific node to avoid in the path
            ghost_positions (list): List of (x, y) positions to avoid
            
        Returns:
            list: List of points forming the path, or None if no path exists
        """
        if ghost_positions is None:
            ghost_positions = []
            
        if start == end:
            return [start]
            
        # Build an adjacency list based on actual roads
        adjacency = {}
        for point in self.intersections:
            if point != avoid_node:  # Don't include the node to avoid if specified
                adjacency[point] = []
            
        # Add connected points based on actual roads
        for road in self.roads:
            a, b = road
            # Skip roads with avoid_node if specified
            if avoid_node and (avoid_node == a or avoid_node == b):
                continue
                
            # Add both directions
            if a in adjacency:
                adjacency[a].append(b)
            if b in adjacency:
                adjacency[b].append(a)
        
        # Check if either start or end is not in adjacency list
        if start not in adjacency or end not in adjacency:
            return None  # No path possible
            
        # Configuration parameters for balancing safety vs. efficiency
        # Higher values prioritize safety over shortest path
        GHOST_PENALTY_WEIGHT = 1.0  # Base penalty weight for ghost proximity
        MAX_PATH_LENGTH_FACTOR = 1.5  # Allow paths up to 1.5x the shortest possible
        
        # First find the shortest path (ignoring ghosts) to use as reference
        # This gives us a baseline for how long the path should be
        shortest_distance = self.estimate_shortest_distance(start, end)
        if shortest_distance == 0:
            shortest_distance = math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
            
        max_allowed_distance = shortest_distance * MAX_PATH_LENGTH_FACTOR
        
        # Heuristic function - balance between distance to goal and ghost avoidance
        def heuristic(node, goal, ghosts):
            # Base distance to goal (straight-line distance)
            dx = node[0] - goal[0]
            dy = node[1] - goal[1]
            base_cost = math.sqrt(dx*dx + dy*dy)
            
            # Add ghost avoidance cost
            ghost_cost = 0
            for ghost_x, ghost_y in ghosts:
                # Calculate distance to ghost
                g_dx = node[0] - ghost_x
                g_dy = node[1] - ghost_y
                distance = math.sqrt(g_dx*g_dx + g_dy*g_dy)
                
                # If too close to ghost, add penalty
                if distance < 200:  # Configurable distance threshold
                    # The closer the ghost, the higher the penalty
                    ghost_cost += (200 - distance) * GHOST_PENALTY_WEIGHT
                    
            return base_cost + ghost_cost
        
        # A* algorithm
        open_set = {start}
        closed_set = set()
        
        # Costs and paths
        g_score = {start: 0}  # Cost from start to node
        f_score = {start: heuristic(start, end, ghost_positions)}  # Estimated total cost
        came_from = {}  # Parent nodes
        real_distance = {start: 0}  # Track actual path distance
        
        while open_set:
            # Find node with lowest f_score
            current = min(open_set, key=lambda node: f_score.get(node, float('inf')))
            
            # Check if we've reached the goal
            if current == end:
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return list(reversed(path))
                
            # Move current from open to closed
            open_set.remove(current)
            closed_set.add(current)
            
            # Explore neighbors
            for neighbor in adjacency.get(current, []):
                if neighbor in closed_set:
                    continue
                    
                # Calculate actual distance traveled
                distance = math.sqrt((current[0] - neighbor[0])**2 + (current[1] - neighbor[1])**2)
                tentative_real_distance = real_distance[current] + distance
                
                # Skip paths that are too long compared to the shortest possible
                if tentative_real_distance > max_allowed_distance:
                    continue
                
                # Calculate tentative g_score (path cost)
                tentative_g = g_score[current] + distance
                
                # Extra cost for neighbors near ghosts - make paths near ghosts more expensive
                ghost_penalty = 0
                for ghost_x, ghost_y in ghost_positions:
                    g_dx = neighbor[0] - ghost_x
                    g_dy = neighbor[1] - ghost_y
                    ghost_distance = math.sqrt(g_dx*g_dx + g_dy*g_dy)
                    
                    # Add penalty for proximity to ghosts
                    if ghost_distance < 200:
                        # The closer the ghost, the higher the penalty
                        # Scale the penalty by distance: closer is much worse
                        proximity_factor = (200 - ghost_distance) / 200
                        ghost_penalty += proximity_factor * GHOST_PENALTY_WEIGHT * distance * 3.0
                
                tentative_g += ghost_penalty
                
                # If neighbor not in open set, add it
                if neighbor not in open_set:
                    open_set.add(neighbor)
                # If this path to neighbor is worse than previous one, skip
                elif tentative_g >= g_score.get(neighbor, float('inf')):
                    continue
                    
                # This path is best so far - record it
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                real_distance[neighbor] = tentative_real_distance
                f_score[neighbor] = tentative_g + heuristic(neighbor, end, ghost_positions)
                
        # No path found
        return None
    
    def estimate_shortest_distance(self, start, end):
        """
        Calculate the shortest possible distance from start to end
        using only the road network (no ghost avoidance)
        
        Args:
            start (tuple): Start point (x, y)
            end (tuple): End point (x, y)
            
        Returns:
            float: Shortest distance, or float('inf') if no path exists
        """
        if start == end:
            return 0
            
        # If there's a direct road, return the straight-line distance
        if self.has_road_between(start, end):
            return math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
            
        # Build adjacency list from road network
        adjacency = {}
        for point in self.intersections:
            adjacency[point] = []
            
        for road in self.roads:
            a, b = road
            if a in adjacency:
                adjacency[a].append(b)
            if b in adjacency:
                adjacency[b].append(a)
        
        # Simple Dijkstra's algorithm to find shortest path
        dist = {start: 0}
        queue = [(0, start)]  # (distance, node)
        visited = set()
        
        while queue:
            (d, current) = min(queue)
            queue.remove((d, current))
            
            if current in visited:
                continue
                
            visited.add(current)
            
            if current == end:
                return d
                
            for neighbor in adjacency.get(current, []):
                if neighbor in visited:
                    continue
                    
                distance = math.sqrt((current[0] - neighbor[0])**2 + (current[1] - neighbor[1])**2)
                new_dist = dist[current] + distance
                
                if neighbor not in dist or new_dist < dist[neighbor]:
                    dist[neighbor] = new_dist
                    queue.append((new_dist, neighbor))
        
        # If no path found, return a large value
        return float('inf')
    
    def find_nearest_intersection(self, point):
        """
        Find the nearest intersection to a given point
        
        Args:
            point (tuple): Point (x, y) to find the nearest intersection to
            
        Returns:
            tuple: Nearest intersection point, or None if no intersections exist
        """
        if not self.intersections:
            return None
            
        closest_dist = float('inf')
        closest_point = None
        
        for intersection in self.intersections:
            dx = point[0] - intersection[0]
            dy = point[1] - intersection[1]
            dist = math.sqrt(dx*dx + dy*dy)
            
            if dist < closest_dist:
                closest_dist = dist
                closest_point = intersection
                
        return closest_point
    
    def generate_random_route(self):
        """
        Generate a valid route along roads for vehicles to follow
        
        Returns:
            list: List of points forming the route
        """
        # Check if we have roads
        if not self.roads or not self.intersections:
            print("No roads or intersections available")
            # Return a default position
            if self.intersections:
                return [self.intersections[0]]
            return [(self.width // 2, self.height // 2)]
            
        # Try to find valid start/end points with a path
        attempts = 0
        max_attempts = min(30, len(self.intersections) * 2)  # Scale with grid size
        
        while attempts < max_attempts:
            attempts += 1
            
            # Select random start and end intersections
            start = random.choice(self.intersections)
            end = random.choice(self.intersections)
            
            # Skip if same point
            if start == end:
                continue
                
            # Find path
            path = self.find_path(start, end)
            
            # Check if path is valid and has at least two points
            if path and len(path) >= 2:
                # Double-check that consecutive points have roads between them
                valid_path = True
                for i in range(len(path) - 1):
                    if not self.has_road_between(path[i], path[i+1]):
                        valid_path = False
                        break
                
                if valid_path:
                    return path
        
        # If we still don't have a valid path, find a single valid road segment
        for road in self.roads:
            return list(road)  # Return a list of the two endpoints
            
        # Last resort fallback - just return a single point
        # This will make the car effectively stay in place
        return [self.intersections[0]]
    
    def calculate_path_safety(self, path, obstacle_positions):
        """
        Calculate how safe a path is based on proximity to obstacles
        
        Args:
            path (list): List of points forming the path
            obstacle_positions (list): List of (x, y) positions of obstacles
            
        Returns:
            float: Safety score (0-100, higher is better)
        """
        if not path or len(path) < 2:
            return 0
            
        safety_score = 100  # Start with perfect score
        
        # Penalize for each obstacle near the path
        for point in path:
            for obstacle_x, obstacle_y in obstacle_positions:
                dx = point[0] - obstacle_x
                dy = point[1] - obstacle_y
                distance = math.sqrt(dx*dx + dy*dy)
                
                # Cars very close to path points are a big penalty
                if distance < 80:  # Very close
                    safety_score -= 20
                # Cars somewhat close are a moderate penalty    
                elif distance < 160:  # Moderately close
                    safety_score -= 10
                # Cars in the general vicinity are a small penalty
                elif distance < 240:  # In vicinity
                    safety_score -= 5
                    
        return max(0, safety_score)  # Don't go below 0
    
    def calculate_path_efficiency(self, path, start_point=None, end_point=None):
        """
        Calculate how efficient a path is (based on directness and length)
        
        Args:
            path (list): List of points forming the path
            start_point (tuple): Optional alternative start point
            end_point (tuple): Optional alternative end point
            
        Returns:
            float: Efficiency score (0-100, higher is better)
        """
        if not path or len(path) < 2:
            return 0
            
        # Use provided start/end or extract from path
        start = start_point if start_point else path[0]
        end = end_point if end_point else path[-1]
        
        # Calculate actual path length
        actual_length = 0
        for i in range(len(path) - 1):
            dx = path[i+1][0] - path[i][0]
            dy = path[i+1][1] - path[i][1]
            segment_length = math.sqrt(dx*dx + dy*dy)
            actual_length += segment_length
            
        # Calculate/estimate shortest possible path
        shortest_length = self.estimate_shortest_distance(start, end)
        if shortest_length == float('inf'):  # Couldn't find a path, use direct distance
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            shortest_length = math.sqrt(dx*dx + dy*dy)
            
        # Calculate efficiency (closer to 100% is better)
        if shortest_length > 0:
            efficiency = min(100, (shortest_length / actual_length) * 100)
        else:
            efficiency = 100  # If both start and end are the same
            
        return efficiency 