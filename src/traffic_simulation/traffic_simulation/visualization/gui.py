import pygame
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Point
import random
import math
from traffic_simulation.visualization.vehicle import VehicleManager
from traffic_simulation.planning.path_finder import PathFinder

class FullscreenKavsakSim(Node):
    def __init__(self):
        super().__init__('fullscreen_kavsak_sim')
        pygame.init()

        self.screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
        self.screen_width, self.screen_height = self.screen.get_size()
        pygame.display.set_caption("Tam Ekran Kavşak Simülasyonu")

        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont('Arial', int(self.screen_height / 30))
        self.bg_color = (34, 139, 34)

        self.dropdown_width = int(self.screen_width / 8)
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
        self.vehicle_spawn_interval = 180  # Frames between vehicle spawns
        self.simulation_speed = 1.0  # Multiplier for simulation speed
        self.pause_simulation = False

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

        for i in range(grid_boyut):
            for j in range(grid_boyut):
                x = (j + 1) * hucre_genislik
                y = (i + 1) * hucre_yukseklik
                self.kavsaklar.append((x, y))

        indeksler = list(range(len(self.kavsaklar)))
        random.shuffle(indeksler)

        baglantilar = set()
        for i in range(len(indeksler)):
            a = self.kavsaklar[indeksler[i]]
            secenekler = [
                (1, 0), (-1, 0), (0, 1), (0, -1),  # düz
                (1, 1), (-1, -1), (-1, 1), (1, -1)  # çapraz
            ]
            random.shuffle(secenekler)
            for dx, dy in secenekler[:random.randint(2, 4)]:
                yeni_i = (a[1] // hucre_yukseklik - 1) + dy
                yeni_j = (a[0] // hucre_genislik - 1) + dx
                if 0 <= yeni_i < grid_boyut and 0 <= yeni_j < grid_boyut:
                    b = self.kavsaklar[yeni_i * grid_boyut + yeni_j]
                    if (a, b) not in baglantilar and (b, a) not in baglantilar:
                        self.yollar.append((a, b))
                        baglantilar.add((a, b))

        sol_kavsaklar = [k for k in self.kavsaklar if k[0] < self.screen_width // 3]
        sag_kavsaklar = [k for k in self.kavsaklar if k[0] > self.screen_width * 2 // 3]
        if sol_kavsaklar and sag_kavsaklar:
            self.start = random.choice(sol_kavsaklar)
            self.finish = random.choice(sag_kavsaklar)
        else:
            self.start = self.kavsaklar[0]
            self.finish = self.kavsaklar[-1]

        self.araba_pozisyon = self.start
        
        # Reset vehicle manager and create main vehicle
        self.vehicle_manager = VehicleManager(self.screen_width, self.screen_height)
        self.main_vehicle = self.vehicle_manager.create_vehicle(self.start, "car", (128, 0, 128))  # Purple car
        
        # Find path for main vehicle
        path = PathFinder.find_path(self.start, self.finish, self.yollar)
        if path:
            smooth_path = PathFinder.smooth_path(path, 5)
            self.main_vehicle.set_route(smooth_path)

        self.publish_map()

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

    def spawn_random_vehicle(self):
        """Spawn a random vehicle at a random intersection"""
        if not self.kavsaklar:
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
            smooth_path = PathFinder.smooth_path(path, 3)
            vehicle.set_route(smooth_path)

    def update(self):
        """Update simulation state"""
        if self.pause_simulation:
            return
            
        # Spawn random vehicles periodically
        self.vehicle_spawn_timer += 1
        if self.vehicle_spawn_timer >= self.vehicle_spawn_interval:
            self.spawn_random_vehicle()
            self.vehicle_spawn_timer = 0
            
        # Update all vehicles
        self.vehicle_manager.update_all()
        
        # Remove inactive vehicles
        self.vehicle_manager.remove_inactive()
        
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
                    dropdown_rect = pygame.Rect(
                        int(self.screen_width * 0.05),
                        int(self.screen_height * 0.05),
                        self.dropdown_width,
                        self.dropdown_height
                    )

                    if dropdown_rect.collidepoint(mouse_pos):
                        self.dropdown_acik = not self.dropdown_acik

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

            # Update simulation
            self.update()

            self.screen.fill(self.bg_color)

            for yol in self.yollar:
                pygame.draw.line(self.screen, (100, 100, 100), yol[0], yol[1], int(self.screen_height * 0.04))
                pygame.draw.line(self.screen, (255, 255, 255), yol[0], yol[1], 2)

            for kavsak in self.kavsaklar:
                pygame.draw.circle(self.screen, (0, 0, 0), kavsak, int(self.screen_height * 0.02))
                pygame.draw.circle(self.screen, (180, 180, 180), kavsak, int(self.screen_height * 0.018))

            if self.start:
                pygame.draw.circle(self.screen, (255, 0, 0), self.start, int(self.screen_height * 0.025))
            if self.finish:
                pygame.draw.circle(self.screen, (0, 0, 255), self.finish, int(self.screen_height * 0.025))
                
            # Draw all vehicles
            self.vehicle_manager.draw_all(self.screen)

            dropdown_rect = pygame.Rect(
                int(self.screen_width * 0.05),
                int(self.screen_height * 0.05),
                self.dropdown_width,
                self.dropdown_height
            )
            pygame.draw.rect(self.screen, (220, 220, 220), dropdown_rect)
            pygame.draw.rect(self.screen, (0, 0, 0), dropdown_rect, 2)
            text = self.font.render(f"Kavşak: {self.secili_kavsak}", True, (0, 0, 0))
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
                    text = self.font.render(str(sayi), True, (0, 0, 0))
                    self.screen.blit(text, (option_rect.x + 10, option_rect.y + 5))
                    
            # Draw simulation status
            status_text = self.font.render(
                f"{'PAUSED' if self.pause_simulation else 'Running'} | Speed: {self.simulation_speed:.1f}x | Vehicles: {len(self.vehicle_manager.vehicles)}",
                True, (255, 255, 255))
            self.screen.blit(status_text, (int(self.screen_width * 0.05), int(self.screen_height * 0.95)))
            
            # Draw controls help
            help_text = self.font.render(
                "Controls: Space=Pause, R=Reset, +/-=Speed, ESC=Exit", 
                True, (255, 255, 255))
            self.screen.blit(help_text, (int(self.screen_width * 0.5), int(self.screen_height * 0.95)))

            pygame.display.flip()
            self.clock.tick(60)

def main(args=None):
    try:
        rclpy.init(args=args)
        sim = FullscreenKavsakSim()
        sim.run()
        sim.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        print(f"Error in GUI node: {e}")
        import traceback
        traceback.print_exc()
        pygame.quit()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
