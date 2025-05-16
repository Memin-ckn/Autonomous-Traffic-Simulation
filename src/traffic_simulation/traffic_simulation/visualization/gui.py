import pygame
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Point
import random
import math
from traffic_simulation.visualization.vehicle_visualization import VehicleManager
from traffic_simulation.planning.path_finder import PathFinder

class FullscreenKavsakSim(Node):
    def __init__(self):
        super().__init__('fullscreen_kavsak_sim')
        pygame.init()

        self.screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
        self.screen_width, self.screen_height = self.screen.get_size()
        pygame.display.set_caption("UPDATED TRAFFIC SIM - VISUAL TEST")

        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont('Arial', int(self.screen_height / 30))
        self.bg_color = (34, 139, 34)  # Match the GREEN from purple_car.py

        # Improve dropdown dimensions
        self.dropdown_width = int(self.screen_width / 6)
        self.dropdown_height = int(self.screen_height / 20)

        self.kavsak_sayilari = [3, 4, 5, 6, 7, 8]
        self.secili_kavsak = 4
        self.dropdown_acik = False

        self.kavsaklar = []
        self.yollar = []
        self.start = None
        self.finish = None
        self.araba_pozisyon = None
        
        # Add vehicle manager
        self.vehicle_manager = VehicleManager(self.screen_width, self.screen_height)
        self.main_vehicle = None
        self.vehicle_spawn_timer = 0
        self.vehicle_spawn_interval = 60  # Even shorter interval for more traffic
        self.max_random_vehicles = 20     # Cap for random vehicles
        self.simulation_speed = 1.0  # Multiplier for simulation speed
        self.pause_simulation = False
        
        # Road width based on screen size (similar to purple_car.py)
        self.road_width = int(self.screen_height * 0.04)  # 4% of screen height

        # ROS2 publishers
        self.map_pub = self.create_publisher(Int32MultiArray, '/map_data', 10)
        self.start_pub = self.create_publisher(Point, '/start', 10)
        self.finish_pub = self.create_publisher(Point, '/finish', 10)

        self.kavsak_olustur()

    def kavsak_olustur(self):
        grid_boyut = self.secili_kavsak
        hucre_genislik = self.screen_width // (grid_boyut + 1)
        hucre_yukseklik = self.screen_height // (grid_boyut + 1)

        self.kavsaklar = []
        self.yollar = []

        # Create grid of intersections
        for i in range(grid_boyut):
            for j in range(grid_boyut):
                x = (j + 1) * hucre_genislik
                y = (i + 1) * hucre_yukseklik
                self.kavsaklar.append((x, y))

        # First, select start and finish points
        sol_kavsaklar = [k for k in self.kavsaklar if k[0] < self.screen_width // 3]
        sag_kavsaklar = [k for k in self.kavsaklar if k[0] > self.screen_width * 2 // 3]
        if sol_kavsaklar and sag_kavsaklar:
            self.start = random.choice(sol_kavsaklar)
            self.finish = random.choice(sag_kavsaklar)
        else:
            self.start = self.kavsaklar[0]
            self.finish = self.kavsaklar[-1]

        # Create a minimum spanning tree to ensure all nodes are connected
        baglantilar = set()
        unvisited = set(self.kavsaklar)
        visited = {self.start}  # Start with the start node
        unvisited.remove(self.start)

        while unvisited:
            # Find the closest unvisited node to any visited node
            min_dist = float('inf')
            best_edge = None
            
            for v in visited:
                for u in unvisited:
                    dist = math.sqrt((v[0] - u[0])**2 + (v[1] - u[1])**2)
                    if dist < min_dist:
                        min_dist = dist
                        best_edge = (v, u)
            
            if best_edge:
                a, b = best_edge
                self.yollar.append((a, b))
                baglantilar.add((a, b))
                baglantilar.add((b, a))
                visited.add(b)
                unvisited.remove(b)

        # Add some additional random connections for variety
        indeksler = list(range(len(self.kavsaklar)))
        random.shuffle(indeksler)
        
        for i in range(len(indeksler)):
            a = self.kavsaklar[indeksler[i]]
            secenekler = [
                (1, 0), (-1, 0), (0, 1), (0, -1),  # straight
                (1, 1), (-1, -1), (-1, 1), (1, -1)  # diagonal
            ]
            random.shuffle(secenekler)
            
            # Add 1-2 random connections per node
            connections_added = 0
            for dx, dy in secenekler:
                if connections_added >= random.randint(1, 2):
                    break
                    
                yeni_i = (a[1] // hucre_yukseklik - 1) + dy
                yeni_j = (a[0] // hucre_genislik - 1) + dx
                
                if 0 <= yeni_i < grid_boyut and 0 <= yeni_j < grid_boyut:
                    b = self.kavsaklar[yeni_i * grid_boyut + yeni_j]
                    if (a, b) not in baglantilar and (b, a) not in baglantilar:
                        self.yollar.append((a, b))
                        baglantilar.add((a, b))
                        baglantilar.add((b, a))
                        connections_added += 1

        self.araba_pozisyon = self.start
        
        # Reset vehicle manager without creating a main vehicle yet
        self.vehicle_manager = VehicleManager(self.screen_width, self.screen_height)
        self.main_vehicle = None
        self.start = None
        self.finish = None
        self.vehicle_spawn_timer = 0
        
        # Initial spawn of some vehicles
        self.spawn_initial_vehicles(5)  # Start with 5 vehicles
        
        # Do not automatically create a path - wait for user selections
        self.publish_map()

    def spawn_initial_vehicles(self, count):
        """Spawn initial vehicles on the map"""
        for _ in range(count):
            self.spawn_random_vehicle()

    def spawn_random_vehicle(self):
        """Spawn a random vehicle at a random intersection"""
        if not self.kavsaklar or len(self.vehicle_manager.vehicles) >= self.max_random_vehicles:
            return
            
        start_node = random.choice(self.kavsaklar)
        possible_ends = [k for k in self.kavsaklar if k != start_node]
        
        if not possible_ends:
            return
            
        end_node = random.choice(possible_ends)
        vehicle = self.vehicle_manager.create_vehicle(start_node)
        
        # Find path for the vehicle
        path = PathFinder.find_path(start_node, end_node, self.yollar)
        if path:
            smooth_path = PathFinder.smooth_path(path, 5)  # More points for smoother path
            vehicle.set_route(smooth_path)
            
            # Randomize the vehicle velocity slightly
            vehicle.state.velocity = random.uniform(4.0, 7.0)
            return True
        
        # If path finding failed, remove the vehicle
        if vehicle.id in self.vehicle_manager.vehicles:
            del self.vehicle_manager.vehicles[vehicle.id]
        return False

    def publish_map(self):
        grid_size = self.secili_kavsak
        grid = [[1 for _ in range(grid_size)] for _ in range(grid_size)]

        for i, (x, y) in enumerate(self.kavsaklar):
            row = (y * grid_size) // self.screen_height
            col = (x * grid_size) // self.screen_width
            grid[row][col] = 0

        flat = [item for row in grid for item in row]
        msg = Int32MultiArray()
        msg.data = flat
        self.map_pub.publish(msg)

        def kavsagi_grid_konumuna_çevir(p):
            return Point(x=float((p[0] * grid_size) / self.screen_width), 
                         y=float((p[1] * grid_size) / self.screen_height), 
                         z=0.0)

        self.start_pub.publish(kavsagi_grid_konumuna_çevir(self.start))
        self.finish_pub.publish(kavsagi_grid_konumuna_çevir(self.finish))

    def update(self):
        """Update simulation state"""
        if self.pause_simulation:
            return
            
        # Calculate delta time based on simulation speed
        dt = self.clock.get_time() / 1000.0 * self.simulation_speed
            
        # Spawn random vehicles periodically
        self.vehicle_spawn_timer += 1
        if self.vehicle_spawn_timer >= self.vehicle_spawn_interval:
            # Try to spawn a new vehicle
            if len(self.vehicle_manager.vehicles) < self.max_random_vehicles:
                self.spawn_random_vehicle()
            self.vehicle_spawn_timer = 0
            
        # Update all vehicles
        self.vehicle_manager.update_all(dt)
        
        # Remove inactive vehicles
        self.vehicle_manager.remove_inactive()
        
        # If we're below the minimum number of vehicles, try to spawn more
        if len(self.vehicle_manager.vehicles) < 5 and not self.pause_simulation:
            self.spawn_random_vehicle()
        
        # Update main vehicle position for ROS2
        if self.main_vehicle and self.main_vehicle.is_active:
            self.araba_pozisyon = self.main_vehicle.position

    def run(self):
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_SPACE:
                        self.pause_simulation = not self.pause_simulation
                    elif event.key == pygame.K_r:
                        self.kavsak_olustur()  # Reset simulation
                    elif event.key == pygame.K_PLUS or event.key == pygame.K_EQUALS:
                        self.simulation_speed = min(3.0, self.simulation_speed + 0.1)
                    elif event.key == pygame.K_MINUS:
                        self.simulation_speed = max(0.1, self.simulation_speed - 0.1)

                if event.type == pygame.MOUSEBUTTONDOWN:
                    mouse_pos = pygame.mouse.get_pos()
                    self.handle_mouse_click(mouse_pos)

            # Update simulation
            self.update()

            self.screen.fill(self.bg_color)

            # Draw roads with better width
            for yol in self.yollar:
                # Thicker gray road with white center line (like purple_car.py)
                pygame.draw.line(self.screen, (100, 100, 100), yol[0], yol[1], self.road_width)
                pygame.draw.line(self.screen, (255, 255, 255), yol[0], yol[1], 2)

            # Draw intersections
            for kavsak in self.kavsaklar:
                pygame.draw.circle(self.screen, (0, 0, 0), kavsak, int(self.screen_height * 0.02))
                pygame.draw.circle(self.screen, (180, 180, 180), kavsak, int(self.screen_height * 0.018))

            # Draw start and finish points with RED and BLUE colors
            if self.start:
                pygame.draw.circle(self.screen, (255, 0, 0), self.start, int(self.screen_height * 0.025))
            if self.finish:
                pygame.draw.circle(self.screen, (0, 0, 255), self.finish, int(self.screen_height * 0.025))
                
            # Draw all vehicles
            self.vehicle_manager.draw_all(self.screen)
            
            # Improved dropdown styling
            dropdown_rect = pygame.Rect(
                int(self.screen_width * 0.05),
                int(self.screen_height * 0.05),
                self.dropdown_width,
                self.dropdown_height
            )
            pygame.draw.rect(self.screen, (220, 220, 220), dropdown_rect)
            pygame.draw.rect(self.screen, (0, 0, 0), dropdown_rect, 2)
            text = self.font.render(f"Grid Size: {self.secili_kavsak}x{self.secili_kavsak}", True, (0, 0, 0))
            self.screen.blit(text, (dropdown_rect.x + 10, dropdown_rect.y + 5))

            if self.dropdown_acik:
                for i, sayi in enumerate(self.kavsak_sayilari):
                    option_rect = pygame.Rect(
                        dropdown_rect.x,
                        dropdown_rect.y + (i + 1) * self.dropdown_height,
                        self.dropdown_width,
                        self.dropdown_height
                    )
                    pygame.draw.rect(self.screen, (200, 200, 200), option_rect)
                    pygame.draw.rect(self.screen, (0, 0, 0), option_rect, 1)
                    text = self.font.render(f"{sayi}x{sayi}", True, (0, 0, 0))
                    self.screen.blit(text, (option_rect.x + 10, option_rect.y + 5))
                    
            # Draw simulation status
            status_text = self.font.render(
                f"{'PAUSED' if self.pause_simulation else 'Running'} | Speed: {self.simulation_speed:.1f}x | Vehicles: {len(self.vehicle_manager.vehicles)}",
                True, (255, 255, 255))
            self.screen.blit(status_text, (int(self.screen_width * 0.05), int(self.screen_height * 0.95)))
            
            # Draw controls help
            help_text = self.font.render(
                "Controls: Space=Pause, R=Reset, +/-=Speed, ESC=Exit | Click intersections to set start/end", 
                True, (255, 255, 255))
            self.screen.blit(help_text, (int(self.screen_width * 0.3), int(self.screen_height * 0.95)))

            pygame.display.flip()
            self.clock.tick(60)

    def create_purple_car(self):
        """Create a purple car at the start position and set its path to the finish"""
        if self.start:
            # Create the vehicle at the start position
            self.main_vehicle = self.vehicle_manager.create_vehicle(self.start)
            
            # Ensure this vehicle is set as the main vehicle with purple color
            self.vehicle_manager.set_main_vehicle(self.main_vehicle.id)
            
            # If finish is already set, calculate path
            if self.finish:
                # Use PathFinder to calculate a proper path
                path = PathFinder.find_path(self.start, self.finish, self.yollar)
                if path:
                    # Create a smoother path with more points
                    smooth_path = PathFinder.smooth_path(path, 10)
                    self.main_vehicle.set_route(smooth_path)
                else:
                    # Fallback to direct path if no path found
                    self.main_vehicle.route = [self.start, self.finish]
                    self.main_vehicle.state.target_index = 0

    def handle_mouse_click(self, mouse_pos):
        """Handle mouse clicks for setting start and finish points"""
        dropdown_rect = pygame.Rect(
            int(self.screen_width * 0.05),
            int(self.screen_height * 0.05),
            self.dropdown_width,
            self.dropdown_height
        )
        
        # Handle dropdown menu clicks
        if dropdown_rect.collidepoint(mouse_pos):
            self.dropdown_acik = not self.dropdown_acik
            return
            
        if self.dropdown_acik:
            for i, sayi in enumerate(self.kavsak_sayilari):
                option_rect = pygame.Rect(
                    dropdown_rect.x,
                    dropdown_rect.y + (i + 1) * self.dropdown_height,
                    self.dropdown_width,
                    self.dropdown_height
                )
                if option_rect.collidepoint(mouse_pos):
                    self.secili_kavsak = sayi
                    self.dropdown_acik = False
                    self.kavsak_olustur()
                    return
            # Click outside dropdown options, close dropdown
            self.dropdown_acik = False
            return
        
        # Check if click is near any intersection
        closest_kavsak = None
        closest_distance = float('inf')
        
        for kavsak in self.kavsaklar:
            distance = math.sqrt((mouse_pos[0] - kavsak[0])**2 + (mouse_pos[1] - kavsak[1])**2)
            if distance < int(self.screen_height * 0.03) and distance < closest_distance:
                closest_kavsak = kavsak
                closest_distance = distance
        
        if closest_kavsak:
            # If clicked on current start/finish node, reset it
            if self.start == closest_kavsak:
                self.start = None
                # Remove main vehicle when start node is cleared
                if self.main_vehicle:
                    if self.main_vehicle.id in self.vehicle_manager.vehicles:
                        del self.vehicle_manager.vehicles[self.main_vehicle.id]
                    self.main_vehicle = None
                return
            elif self.finish == closest_kavsak:
                self.finish = None
                # If main vehicle exists, clear its route
                if self.main_vehicle:
                    self.main_vehicle.route = []
                    self.main_vehicle.state.target_index = 0
                return
                
            # Set as start or finish based on what's not set yet
            if not self.start:
                self.start = closest_kavsak
                # Create purple car when start is set
                self.create_purple_car()
            elif not self.finish:
                self.finish = closest_kavsak
                # Update car's path when finish is set
                if self.main_vehicle:
                    # Use PathFinder to create a proper path
                    path = PathFinder.find_path(self.start, self.finish, self.yollar)
                    if path:
                        smooth_path = PathFinder.smooth_path(path, 10)
                        self.main_vehicle.set_route(smooth_path)
                    else:
                        # Fallback to direct path
                        self.main_vehicle.route = [self.start, self.finish]
                        self.main_vehicle.state.target_index = 0

def main(args=None):
    print("Starting Traffic Simulation...")
    rclpy.init(args=args)
    print("ROS2 initialized")
    
    try:
        print("Creating FullscreenKavsakSim node...")
        node = FullscreenKavsakSim()
        print("Node created successfully")
        
        print("Running the simulation...")
        node.run()
    except Exception as e:
        print(f"Error occurred: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("Shutting down...")
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
        pygame.quit()

if __name__ == '__main__':
    main()
