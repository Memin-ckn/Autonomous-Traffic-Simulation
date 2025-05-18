import heapq
import math

class PathFinder:
    """Simple path finding implementation using Dijkstra's algorithm"""
    
    @staticmethod
    def find_path(start, end, roads):
        """
        Find the shortest path from start to end using the given roads
        
        Parameters:
        - start: Starting coordinate tuple (x, y)
        - end: Ending coordinate tuple (x, y)
        - roads: List of roads as tuple pairs ((x1, y1), (x2, y2))
        
        Returns:
        - List of coordinate points to follow or None if no path exists
        """
        
        # Build road network graph
        graph = {}
        for road in roads:
            a, b = road
            if a not in graph:
                graph[a] = []
            if b not in graph:
                graph[b] = []
            graph[a].append(b)
            graph[b].append(a)  # Bidirectional roads
        
        if start not in graph or end not in graph:
            return None
            
        # Dijkstra's algorithm implementation
        distances = {node: float('infinity') for node in graph}
        distances[start] = 0
        priority_queue = [(0, start)]
        previous = {node: None for node in graph}
        
        while priority_queue:
            current_distance, current_node = heapq.heappop(priority_queue)
            
            if current_node == end:
                # Reconstruct path from end to start
                path = []
                while current_node:
                    path.append(current_node)
                    current_node = previous[current_node]
                return list(reversed(path))
                
            if current_distance > distances[current_node]:
                continue
                
            for neighbor in graph[current_node]:
                distance = PathFinder._calculate_distance(current_node, neighbor)
                distance = current_distance + distance
                
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous[neighbor] = current_node
                    heapq.heappush(priority_queue, (distance, neighbor))
        
        return None
    
    @staticmethod
    def _calculate_distance(a, b):
        """Calculate Euclidean distance between two points"""
        return math.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)
        
    @staticmethod
    def smooth_path(path, num_points=10):
        """
        Create a smoother path by adding intermediate points
        This makes vehicle movement more fluid
        """
        # Add intermediate points for smoother vehicle movement
        if not path or len(path) < 2:
            return path
            
        smooth_path = []
        for i in range(len(path) - 1):
            a = path[i]
            b = path[i + 1]
            smooth_path.append(a)
            
            # Linear interpolation between points
            for j in range(1, num_points):
                t = j / num_points
                x = a[0] + t * (b[0] - a[0])
                y = a[1] + t * (b[1] - a[1])
                smooth_path.append((x, y))
                
        smooth_path.append(path[-1])
        return smooth_path 