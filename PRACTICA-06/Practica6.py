import time
import math
import numpy as np
import pygame
from pyrplidar import PyRPlidar
import signal
import sys
from collections import deque
import random

# Configuración del LIDAR
MIN_DISTANCE = 50      # mm (5 cm)
MAX_DISTANCE = 3000    # mm (300 cm)
OBSTACLE_THRESHOLD = 800  # mm (80 cm)
FRONT_ANGLE_RANGE = 60  # grados a cada lado del frente

# Configuración de motores
try:
    import smbus2 as smbus
    bus = smbus.SMBus(1)
    bus.write_quick(0x34)
    print("[OK] I2C conectado")
except Exception as e:
    print(f"[Simulación] I2C no disponible: {e}")
    class FakeBus:
        def write_i2c_block_data(self, addr, reg, data):
            print(f"[Simulación] I2C -> Addr: {hex(addr)}, Reg: {hex(reg)}, Data: {data}")
    bus = FakeBus()

DIRECCION_MOTORES = 0x34
REG_VELOCIDAD_FIJA = 0x33

# Parámetros cinemáticos
R = 0.048
l1 = 0.097
l2 = 0.109
W = (1 / R) * np.array([
    [1, -1, (l1 + l2)],
    [1, 1, (l1 + l2)],
    [1, 1, -(l1 + l2)],
    [1, -1, -(l1 + l2)]
])
V_MAX = 250
PWM_MAX = 100

# Pantalla para visualización
WIDTH, HEIGHT = 1200, 800
MAP_SIZE = 600  # Tamaño del área de mapeo
CENTER = (MAP_SIZE // 2, HEIGHT // 2)
SCALE = (MAP_SIZE // 2) / MAX_DISTANCE
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
YELLOW = (255, 255, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
CYAN = (0, 255, 255)
MAGENTA = (255, 0, 255)
DARK_GREEN = (0, 100, 0)
WHITE = (255, 255, 255)
GRAY = (128, 128, 128)

# Variables globales
running = True

# ============= OCUPANCY GRID MAP =============
class OccupancyGrid:
    """Mapa de ocupación para representar el entorno"""
    def __init__(self, width_mm, height_mm, resolution_mm=50):
        """
        width_mm, height_mm: dimensiones del mapa en mm
        resolution_mm: tamaño de cada celda en mm
        """
        self.resolution = resolution_mm
        self.width = int(width_mm / resolution_mm)
        self.height = int(height_mm / resolution_mm)
        self.grid = np.full((self.height, self.width), -1, dtype=np.int8)  # -1: desconocido, 0: libre, 100: ocupado
        self.origin_x = self.width // 2
        self.origin_y = self.height // 2
        
    def world_to_grid(self, x_mm, y_mm):
        """Convierte coordenadas del mundo (mm) a índices de grid"""
        grid_x = int(x_mm / self.resolution) + self.origin_x
        grid_y = int(-y_mm / self.resolution) + self.origin_y
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x, grid_y):
        """Convierte índices de grid a coordenadas del mundo (mm)"""
        x_mm = (grid_x - self.origin_x) * self.resolution
        y_mm = -(grid_y - self.origin_y) * self.resolution
        return x_mm, y_mm
    
    def is_valid(self, grid_x, grid_y):
        """Verifica si las coordenadas están dentro del grid"""
        return 0 <= grid_x < self.width and 0 <= grid_y < self.height
    
    def is_free(self, grid_x, grid_y):
        """Verifica si una celda está libre"""
        if not self.is_valid(grid_x, grid_y):
            return False
        return self.grid[grid_y, grid_x] == 0
    
    def is_occupied(self, grid_x, grid_y):
        """Verifica si una celda está ocupada"""
        if not self.is_valid(grid_x, grid_y):
            return True
        return self.grid[grid_y, grid_x] == 100
    
    def update_from_lidar(self, robot_x, robot_y, robot_angle, lidar_points):
        """Actualiza el mapa con datos del LIDAR usando ray tracing"""
        robot_gx, robot_gy = self.world_to_grid(robot_x, robot_y)
        
        for angle, distance in lidar_points:
            if distance < MIN_DISTANCE or distance > MAX_DISTANCE:
                continue
            
            # Calcular posición del punto en coordenadas del mundo
            angle_rad = math.radians(angle + robot_angle)
            point_x = robot_x + distance * math.cos(angle_rad)
            point_y = robot_y + distance * math.sin(angle_rad)
            
            point_gx, point_gy = self.world_to_grid(point_x, point_y)
            
            # Ray tracing: marcar celdas libres desde robot hasta el obstáculo
            for cell_x, cell_y in self._bresenham_line(robot_gx, robot_gy, point_gx, point_gy):
                if self.is_valid(cell_x, cell_y):
                    if (cell_x, cell_y) == (point_gx, point_gy):
                        # Punto final: obstáculo
                        if distance <= OBSTACLE_THRESHOLD * 1.2:  # Obstáculos cercanos más confiables
                            self.grid[cell_y, cell_x] = 100
                    else:
                        # Camino libre
                        if self.grid[cell_y, cell_x] != 100:
                            self.grid[cell_y, cell_x] = 0
    
    def _bresenham_line(self, x0, y0, x1, y1):
        """Algoritmo de Bresenham para trazar líneas en el grid"""
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        while True:
            cells.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        
        return cells

# ============= RRT =============
class RRTNode:
    """Nodo del árbol RRT"""
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0

class RRT:
    """Implementación de RRT para exploración autónoma"""
    def __init__(self, occupancy_grid, step_size=200, goal_sample_rate=0.1):
        """
        occupancy_grid: mapa de ocupación
        step_size: distancia máxima entre nodos (mm)
        goal_sample_rate: probabilidad de muestrear hacia fronteras
        """
        self.grid = occupancy_grid
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.nodes = []
        self.frontiers = []
        
    def plan(self, start_x, start_y, max_iterations=500):
        """
        Planifica un camino hacia una frontera inexplorada
        Retorna: lista de waypoints [(x, y), ...] o None
        """
        self.nodes = [RRTNode(start_x, start_y)]
        self.update_frontiers()
        
        if not self.frontiers:
            print("No hay fronteras por explorar")
            return None
        
        for i in range(max_iterations):
            # Muestreo
            if random.random() < self.goal_sample_rate and self.frontiers:
                # Muestrear cerca de una frontera
                frontier = random.choice(self.frontiers)
                rand_x = frontier[0] + random.uniform(-300, 300)
                rand_y = frontier[1] + random.uniform(-300, 300)
            else:
                # Muestreo aleatorio
                rand_x = random.uniform(-2000, 2000)
                rand_y = random.uniform(-2000, 2000)
            
            # Encontrar nodo más cercano
            nearest_node = self.get_nearest_node(rand_x, rand_y)
            
            # Extender hacia el punto aleatorio
            new_node = self.steer(nearest_node, rand_x, rand_y)
            
            # Verificar colisiones
            if self.is_collision_free(nearest_node, new_node):
                new_node.parent = nearest_node
                new_node.cost = nearest_node.cost + self.distance(nearest_node, new_node)
                self.nodes.append(new_node)
                
                # Verificar si llegamos a una frontera
                if self.is_near_frontier(new_node, threshold=300):
                    print(f"Camino encontrado en {i+1} iteraciones")
                    return self.extract_path(new_node)
        
        # Si no se llegó a una frontera, devolver el mejor nodo
        best_node = self.get_best_exploration_node()
        if best_node:
            print(f"Retornando mejor nodo de exploración")
            return self.extract_path(best_node)
        
        return None
    
    def get_nearest_node(self, x, y):
        """Encuentra el nodo más cercano a (x, y)"""
        min_dist = float('inf')
        nearest = self.nodes[0]
        for node in self.nodes:
            dist = math.sqrt((node.x - x)**2 + (node.y - y)**2)
            if dist < min_dist:
                min_dist = dist
                nearest = node
        return nearest
    
    def steer(self, from_node, to_x, to_y):
        """Crea un nuevo nodo en dirección a (to_x, to_y) con step_size máximo"""
        dx = to_x - from_node.x
        dy = to_y - from_node.y
        dist = math.sqrt(dx**2 + dy**2)
        
        if dist <= self.step_size:
            return RRTNode(to_x, to_y)
        else:
            theta = math.atan2(dy, dx)
            new_x = from_node.x + self.step_size * math.cos(theta)
            new_y = from_node.y + self.step_size * math.sin(theta)
            return RRTNode(new_x, new_y)
    
    def is_collision_free(self, from_node, to_node):
        """Verifica si el camino entre dos nodos está libre de colisiones"""
        steps = int(self.distance(from_node, to_node) / (self.grid.resolution / 2))
        if steps == 0:
            steps = 1
        
        for i in range(steps + 1):
            t = i / steps
            x = from_node.x + t * (to_node.x - from_node.x)
            y = from_node.y + t * (to_node.y - from_node.y)
            
            gx, gy = self.grid.world_to_grid(x, y)
            if self.grid.is_occupied(gx, gy):
                return False
            
            # Verificar margen de seguridad
            safety_margin = 2  # celdas
            for dx in range(-safety_margin, safety_margin + 1):
                for dy in range(-safety_margin, safety_margin + 1):
                    if self.grid.is_occupied(gx + dx, gy + dy):
                        return False
        
        return True
    
    def distance(self, node1, node2):
        """Calcula distancia euclidiana entre dos nodos"""
        return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)
    
    def update_frontiers(self):
        """Actualiza las celdas frontera (límite entre conocido y desconocido)"""
        self.frontiers = []
        
        for y in range(1, self.grid.height - 1):
            for x in range(1, self.grid.width - 1):
                # Una frontera es una celda libre adyacente a una desconocida
                if self.grid.grid[y, x] == 0:
                    for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
                        nx, ny = x + dx, y + dy
                        if self.grid.grid[ny, nx] == -1:
                            world_x, world_y = self.grid.grid_to_world(x, y)
                            self.frontiers.append((world_x, world_y))
                            break
        
        print(f"Fronteras detectadas: {len(self.frontiers)}")
    
    def is_near_frontier(self, node, threshold=300):
        """Verifica si un nodo está cerca de una frontera"""
        for fx, fy in self.frontiers:
            dist = math.sqrt((node.x - fx)**2 + (node.y - fy)**2)
            if dist < threshold:
                return True
        return False
    
    def get_best_exploration_node(self):
        """Obtiene el nodo que maximiza la exploración"""
        best_node = None
        best_score = -float('inf')
        
        for node in self.nodes:
            if node.parent is None:
                continue
            
            # Calcular score: minimizar distancia a fronteras
            min_frontier_dist = float('inf')
            for fx, fy in self.frontiers:
                dist = math.sqrt((node.x - fx)**2 + (node.y - fy)**2)
                if dist < min_frontier_dist:
                    min_frontier_dist = dist
            
            score = -min_frontier_dist - node.cost * 0.1  # Preferir nodos cercanos a fronteras
            
            if score > best_score:
                best_score = score
                best_node = node
        
        return best_node
    
    def extract_path(self, goal_node):
        """Extrae el camino desde el inicio hasta el nodo objetivo"""
        path = []
        node = goal_node
        while node is not None:
            path.append((node.x, node.y))
            node = node.parent
        path.reverse()
        return path

# ============= CONTROL DE MOTORES =============
def calcular_pwm(vx, vy, omega):
    """Calcula PWM para cada motor"""
    V = np.array([vx, vy, omega])
    velocidades = np.dot(W, V)
    factor_escala = np.max(np.abs(velocidades)) / 250 if np.max(np.abs(velocidades)) > 250 else 1
    if factor_escala > 1:
        velocidades /= factor_escala
    velocidades[1] *= -1
    velocidades[2] *= -1
    pwm = np.clip((velocidades / V_MAX) * PWM_MAX, -PWM_MAX, PWM_MAX)
    return [int(p) for p in pwm]

def enviar_pwm(vx, vy, omega):
    """Envía comandos PWM a los motores"""
    pwm = calcular_pwm(vx, vy, omega)
    bus.write_i2c_block_data(DIRECCION_MOTORES, REG_VELOCIDAD_FIJA, pwm)

def detener_motores():
    """Detiene todos los motores"""
    try:
        bus.write_i2c_block_data(DIRECCION_MOTORES, REG_VELOCIDAD_FIJA, [0, 0, 0, 0])
        print("Motores detenidos")
    except Exception as e:
        print(f"Error al detener motores: {e}")

def cerrar_todo(signal_received=None, frame=None):
    """Limpieza al cerrar el programa"""
    global running
    running = False
    detener_motores()
    print("\nPrograma terminado")
    sys.exit(0)

signal.signal(signal.SIGINT, cerrar_todo)
signal.signal(signal.SIGTERM, cerrar_todo)

# ============= CONTROL DE NAVEGACIÓN =============
class PathFollower:
    """Controlador para seguir un camino"""
    def __init__(self, path):
        self.path = path
        self.current_waypoint = 0
        self.waypoint_tolerance = 150  # mm
        
    def get_control(self, robot_x, robot_y, robot_angle):
        """Calcula comandos de control para seguir el camino"""
        if self.current_waypoint >= len(self.path):
            return 0, 0, 0, True  # Camino completado
        
        target_x, target_y = self.path[self.current_waypoint]
        
        # Calcular distancia y ángulo al waypoint
        dx = target_x - robot_x
        dy = target_y - robot_y
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.degrees(math.atan2(dy, dx))
        
        # Error angular
        angle_error = target_angle - robot_angle
        angle_error = (angle_error + 180) % 360 - 180  # Normalizar a [-180, 180]
        
        # Verificar si llegamos al waypoint
        if distance < self.waypoint_tolerance:
            self.current_waypoint += 1
            print(f"Waypoint {self.current_waypoint}/{len(self.path)} alcanzado")
            return 0, 0, 0, False
        
        # Control proporcional
        vx = min(150, distance * 0.5)  # Velocidad proporcional a distancia
        omega = angle_error * 0.8  # Control angular
        
        # Reducir velocidad lineal si el error angular es grande
        if abs(angle_error) > 30:
            vx *= 0.3
        
        return vx, 0, omega, False

# ============= FUNCIÓN PRINCIPAL =============
def main():
    """Función principal del sistema autónomo con RRT"""
    global running
    
    # Inicializar pygame
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("RRT Autonomous Exploration & Mapping")
    clock = pygame.time.Clock()
    font_small = pygame.font.SysFont(None, 18)
    font_title = pygame.font.SysFont(None, 24, bold=True)
    
    # Inicializar LIDAR
    lidar = PyRPlidar()
    lidar.connect(port="/dev/ttyUSB0", baudrate=460800, timeout=3)
    lidar.set_motor_pwm(500)
    time.sleep(2)
    scan_generator = lidar.force_scan()
    
    # Inicializar mapa de ocupación
    occupancy_map = OccupancyGrid(width_mm=6000, height_mm=6000, resolution_mm=50)
    
    # Inicializar RRT
    rrt = RRT(occupancy_map, step_size=200)
    
    # Estado del robot (coordenadas globales)
    robot_pose = {'x': 0, 'y': 0, 'angle': 0}  # mm, mm, grados
    
    # Control de navegación
    path_follower = None
    estado = "mapeando"  # Estados: mapeando, planificando, navegando, explorado
    tiempo_mapeo_inicial = time.time()
    TIEMPO_MAPEO_INICIAL = 5  # segundos para mapeo inicial
    
    print("Sistema de exploración autónoma iniciado")
    print("Fase 1: Mapeo inicial rotando 360°")
    
    try:
        points = []
        prev_angle = None
        replan_counter = 0
        
        for scan in scan_generator():
            # Eventos de pygame
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
            
            if not running:
                break
            
            # Recolectar puntos del LIDAR
            if MIN_DISTANCE <= scan.distance <= MAX_DISTANCE:
                points.append((scan.angle, scan.distance))
            
            # Detectar completación de barrido
            if prev_angle is not None and scan.angle < prev_angle:
                # Actualizar mapa
                occupancy_map.update_from_lidar(
                    robot_pose['x'], robot_pose['y'], robot_pose['angle'], points
                )
                
                # Máquina de estados
                if estado == "mapeando":
                    # Mapeo inicial: rotar en su lugar
                    if time.time() - tiempo_mapeo_inicial < TIEMPO_MAPEO_INICIAL:
                        enviar_pwm(0, 0, 30)  # Rotar lentamente
                    else:
                        print("\nFase 2: Iniciando exploración autónoma con RRT")
                        detener_motores()
                        estado = "planificando"
                        time.sleep(0.5)
                
                elif estado == "planificando":
                    # Planificar camino con RRT
                    print("Planificando camino con RRT...")
                    path = rrt.plan(robot_pose['x'], robot_pose['y'], max_iterations=500)
                    
                    if path and len(path) > 1:
                        path_follower = PathFollower(path)
                        estado = "navegando"
                        print(f"Camino planificado con {len(path)} waypoints")
                    else:
                        print("Exploración completada - No hay más fronteras")
                        estado = "explorado"
                        detener_motores()
                
                elif estado == "navegando":
                    # Seguir el camino planificado
                    vx, vy, omega, completed = path_follower.get_control(
                        robot_pose['x'], robot_pose['y'], robot_pose['angle']
                    )
                    
                    if completed:
                        print("Camino completado")
                        detener_motores()
                        estado = "planificando"
                        replan_counter = 0
                    else:
                        enviar_pwm(vx, vy, omega)
                        
                        # Re-planificar periódicamente
                        replan_counter += 1
                        if replan_counter > 50:  # Cada ~50 barridos
                            print("Re-planificando...")
                            estado = "planificando"
                            replan_counter = 0
                
                elif estado == "explorado":
                    detener_motores()
                
                # Actualizar pose del robot (simplificado - en realidad necesitarías odometría)
                # Aquí asumimos que el robot se mueve en la dirección de su orientación
                if estado == "navegando" and path_follower:
                    # Actualización simplificada de pose basada en comandos
                    pass  # En un sistema real, usarías encoders/IMU
                
                # ============= VISUALIZACIÓN =============
                screen.fill(BLACK)
                
                # Panel izquierdo: Vista LIDAR
                lidar_panel_rect = pygame.Rect(0, 0, MAP_SIZE, HEIGHT)
                pygame.draw.rect(screen, DARK_GREEN, lidar_panel_rect, 1)
                
                # Dibujar círculos de referencia
                for r in range(500, MAX_DISTANCE + 1, 500):
                    pygame.draw.circle(screen, DARK_GREEN, CENTER, int(r * SCALE), 1)
                
                # Dibujar puntos LIDAR
                for ang, dist in points:
                    angle_rad = math.radians(ang)
                    r = dist * SCALE
                    px = CENTER[0] + int(r * math.cos(angle_rad))
                    py = CENTER[1] - int(r * math.sin(angle_rad))
                    
                    if dist <= OBSTACLE_THRESHOLD:
                        color = RED
                    elif dist <= 2000:
                        color = YELLOW
                    else:
                        color = GREEN
                    
                    pygame.draw.circle(screen, color, (px, py), 2)
                
                # Dibujar robot
                pygame.draw.circle(screen, CYAN, CENTER, 8)
                
                # Panel derecho: Mapa de ocupación y RRT
                map_panel_x = MAP_SIZE + 20
                map_panel_width = WIDTH - MAP_SIZE - 40
                map_panel_height = HEIGHT - 100
                
                # Título del mapa
                map_title = font_title.render("Occupancy Grid & RRT", True, WHITE)
                screen.blit(map_title, (map_panel_x, 10))
                
                # Dibujar mapa de ocupación
                cell_size = min(map_panel_width // occupancy_map.width, 
                               map_panel_height // occupancy_map.height)
                
                map_start_x = map_panel_x
                map_start_y = 40
                
                for y in range(occupancy_map.height):
                    for x in range(occupancy_map.width):
                        value = occupancy_map.grid[y, x]
                        if value == -1:
                            color = GRAY  # Desconocido
                        elif value == 0:
                            color = WHITE  # Libre
                        else:
                            color = BLACK  # Ocupado
                        
                        rect = pygame.Rect(
                            map_start_x + x * cell_size,
                            map_start_y + y * cell_size,
                            cell_size, cell_size
                        )
                        pygame.draw.rect(screen, color, rect)
                
                # Dibujar fronteras
                for fx, fy in rrt.frontiers[:100]:  # Limitar para rendimiento
                    gx, gy = occupancy_map.world_to_grid(fx, fy)
                    px = map_start_x + gx * cell_size
                    py = map_start_y + gy * cell_size
                    pygame.draw.circle(screen, BLUE, (int(px), int(py)), 2)
                
                # Dibujar árbol RRT
                for node in rrt.nodes:
                    if node.parent:
                        gx1, gy1 = occupancy_map.world_to_grid(node.parent.x, node.parent.y)
                        gx2, gy2 = occupancy_map.world_to_grid(node.x, node.y)
                        px1 = map_start_x + gx1 * cell_size
                        py1 = map_start_y + gy1 * cell_size
                        px2 = map_start_x + gx2 * cell_size
                        py2 = map_start_y + gy2 * cell_size
                        pygame.draw.line(screen, CYAN, (px1, py1), (px2, py2), 1)
                
                # Dibujar camino planificado
                if path_follower and path_follower.path:
                    for i in range(len(path_follower.path) - 1):
                        wx1, wy1 = path_follower.path[i]
                        wx2, wy2 = path_follower.path[i + 1]
                        gx1, gy1 = occupancy_map.world_to_grid(wx1, wy1)
                        gx2, gy2 = occupancy_map.world_to_grid(wx2, wy2)
                        px1 = map_start_x + gx1 * cell_size
                        py1 = map_start_y + gy1 * cell_size
                        px2 = map_start_x + gx2 * cell_size
                        py2 = map_start_y + gy2 * cell_size
                        pygame.draw.line(screen, MAGENTA, (px1, py1), (px2, py2), 3)
                
                # Dibujar posición del robot en el mapa
                robot_gx, robot_gy = occupancy_map.world_to_grid(robot_pose['x'], robot_pose['y'])
                robot_px = map_start_x + robot_gx * cell_size
                robot_py = map_start_y + robot_gy * cell_size
                pygame.draw.circle(screen, RED, (int(robot_px), int(robot_py)), 5)
                
                # Información de estado
                info_y = HEIGHT - 80
                status_text = f"Estado: {estado.upper()}"
                status_surface = font_title.render(status_text, True, YELLOW)
                screen.blit(status_surface, (20, info_y))
                
                info_texts = [
                    f"Nodos RRT: {len(rrt.nodes)}",
                    f"Fronteras: {len(rrt.frontiers)}",
                    f"Pose: ({int(robot_pose['x'])}, {int(robot_pose['y'])}, {int(robot_pose['angle'])}°)",
                ]
                
                if path_follower:
                    info_texts.append(f"Waypoint: {path_follower.current_waypoint}/{len(path_follower.path)}")
                
                for i, text in enumerate(info_texts):
                    surface = font_small.render(text, True, WHITE)
                    screen.blit(surface, (20, info_y + 25 + i * 18))
                
                pygame.display.flip()
                clock.tick(60)
                points.clear()
            
            prev_angle = scan.angle
    
    except KeyboardInterrupt:
        print("\nInterrumpido por usuario")
    finally:
        detener_motores()
        lidar.stop()
        lidar.set_motor_pwm(0)
        lidar.disconnect()
        pygame.quit()
        print("Sistema cerrado correctamente")

if __name__ == "__main__":
    main()
