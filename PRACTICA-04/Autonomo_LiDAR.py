import time
import math
import numpy as np
import pygame
from pyrplidar import PyRPlidar
import signal
import sys
from collections import deque
import random

# ============================================================================
# CONFIGURACIÓN DEL LIDAR
# ============================================================================
MIN_DISTANCE = 50      # mm (5 cm)
MAX_DISTANCE = 3000    # mm (300 cm)
OBSTACLE_THRESHOLD = 600  # mm (60 cm) - distancia de seguridad

# ============================================================================
# CONFIGURACIÓN DE MOTORES
# ============================================================================
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

# ============================================================================
# CONFIGURACIÓN DE RRT
# ============================================================================
RRT_MAX_ITERATIONS = 50  # Iteraciones de RRT por ciclo
RRT_STEP_SIZE = 300      # mm - tamaño de paso del árbol
RRT_GOAL_BIAS = 0.15     # Probabilidad de muestrear hacia un objetivo
RRT_SAFE_MARGIN = 150    # mm - margen de seguridad adicional

# ============================================================================
# CONFIGURACIÓN DE VISUALIZACIÓN
# ============================================================================
WIDTH, HEIGHT = 900, 900
CENTER = (WIDTH // 2, HEIGHT // 2)
SCALE = (WIDTH // 2) / MAX_DISTANCE
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
YELLOW = (255, 255, 0)
RED = (255, 0, 0)
BLUE = (0, 150, 255)
CYAN = (0, 255, 255)
MAGENTA = (255, 0, 255)
DARK_GREEN = (0, 100, 0)
WHITE = (255, 255, 255)
GRAY = (100, 100, 100)

# Variables globales
running = True

# ============================================================================
# CLASES DEL SISTEMA
# ============================================================================

class Node:
    """Nodo del árbol RRT"""
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0
    
    def __repr__(self):
        return f"Node({self.x:.0f}, {self.y:.0f})"

class OccupancyGrid:
    """Mapa de ocupación basado en datos del LIDAR"""
    def __init__(self, size_mm=3000, resolution=50):
        """
        size_mm: tamaño del mapa en mm
        resolution: tamaño de cada celda en mm
        """
        self.size_mm = size_mm
        self.resolution = resolution
        self.grid_size = int(size_mm / resolution)
        self.grid = np.zeros((self.grid_size, self.grid_size), dtype=np.uint8)
        self.center = self.grid_size // 2
        
    def world_to_grid(self, x_mm, y_mm):
        """Convierte coordenadas del mundo (mm) a índices de la grilla"""
        grid_x = int(x_mm / self.resolution) + self.center
        grid_y = int(-y_mm / self.resolution) + self.center
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x, grid_y):
        """Convierte índices de la grilla a coordenadas del mundo (mm)"""
        x_mm = (grid_x - self.center) * self.resolution
        y_mm = -(grid_y - self.center) * self.resolution
        return x_mm, y_mm
    
    def is_valid(self, grid_x, grid_y):
        """Verifica si las coordenadas están dentro de la grilla"""
        return 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size
    
    def update_from_lidar(self, points):
        """Actualiza el mapa con datos del LIDAR"""
        # Decaimiento temporal - los obstáculos se desvanecen gradualmente
        self.grid = np.maximum(0, self.grid - 1)
        
        # Marcar obstáculos detectados
        for angle_deg, distance_mm in points:
            if distance_mm < MIN_DISTANCE or distance_mm > MAX_DISTANCE:
                continue
            
            angle_rad = math.radians(angle_deg)
            x_mm = distance_mm * math.cos(angle_rad)
            y_mm = distance_mm * math.sin(angle_rad)
            
            grid_x, grid_y = self.world_to_grid(x_mm, y_mm)
            
            if self.is_valid(grid_x, grid_y):
                self.grid[grid_y, grid_x] = 255  # Marcar como ocupado
                
                # Inflar obstáculos para seguridad
                margin_cells = int(RRT_SAFE_MARGIN / self.resolution)
                for dx in range(-margin_cells, margin_cells + 1):
                    for dy in range(-margin_cells, margin_cells + 1):
                        nx, ny = grid_x + dx, grid_y + dy
                        if self.is_valid(nx, ny):
                            if dx*dx + dy*dy <= margin_cells*margin_cells:
                                self.grid[ny, nx] = max(self.grid[ny, nx], 200)
    
    def is_collision_free(self, x1_mm, y1_mm, x2_mm, y2_mm):
        """Verifica si la línea entre dos puntos está libre de obstáculos"""
        gx1, gy1 = self.world_to_grid(x1_mm, y1_mm)
        gx2, gy2 = self.world_to_grid(x2_mm, y2_mm)
        
        # Algoritmo de Bresenham para verificar todos los puntos de la línea
        points = self.bresenham_line(gx1, gy1, gx2, gy2)
        
        for gx, gy in points:
            if not self.is_valid(gx, gy):
                return False
            if self.grid[gy, gx] > 100:  # Umbral de ocupación
                return False
        
        return True
    
    def bresenham_line(self, x0, y0, x1, y1):
        """Algoritmo de Bresenham para obtener puntos de una línea"""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            points.append((x, y))
            
            if x == x1 and y == y1:
                break
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        
        return points

class RRTPlanner:
    """Planificador RRT para navegación autónoma"""
    def __init__(self, occupancy_grid):
        self.grid = occupancy_grid
        self.nodes = []
        self.reset()
    
    def reset(self):
        """Reinicia el árbol RRT desde el origen"""
        self.nodes = [Node(0, 0)]  # Robot empieza en el origen
    
    def get_random_point(self):
        """Genera un punto aleatorio en el espacio libre"""
        max_range = self.grid.size_mm // 2
        
        # Con cierta probabilidad, muestrear en dirección frontal
        if random.random() < RRT_GOAL_BIAS:
            # Sesgo hacia adelante
            angle = random.uniform(-60, 60)  # Frente
            distance = random.uniform(500, max_range * 0.8)
        else:
            # Muestreo uniforme
            angle = random.uniform(0, 360)
            distance = random.uniform(200, max_range * 0.9)
        
        angle_rad = math.radians(angle)
        x = distance * math.cos(angle_rad)
        y = distance * math.sin(angle_rad)
        
        return x, y
    
    def get_nearest_node(self, x, y):
        """Encuentra el nodo más cercano al punto dado"""
        min_dist = float('inf')
        nearest = None
        
        for node in self.nodes:
            dist = math.sqrt((node.x - x)**2 + (node.y - y)**2)
            if dist < min_dist:
                min_dist = dist
                nearest = node
        
        return nearest
    
    def steer(self, from_node, to_x, to_y):
        """Genera un nuevo nodo en dirección al objetivo, con paso limitado"""
        dx = to_x - from_node.x
        dy = to_y - from_node.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance <= RRT_STEP_SIZE:
            return Node(to_x, to_y)
        
        # Limitar el paso
        ratio = RRT_STEP_SIZE / distance
        new_x = from_node.x + dx * ratio
        new_y = from_node.y + dy * ratio
        
        return Node(new_x, new_y)
    
    def extend_tree(self):
        """Intenta extender el árbol RRT con un nuevo nodo"""
        # Generar punto aleatorio
        rand_x, rand_y = self.get_random_point()
        
        # Encontrar nodo más cercano
        nearest = self.get_nearest_node(rand_x, rand_y)
        
        # Generar nuevo nodo en dirección al punto aleatorio
        new_node = self.steer(nearest, rand_x, rand_y)
        
        # Verificar si el camino está libre de colisiones
        if self.grid.is_collision_free(nearest.x, nearest.y, new_node.x, new_node.y):
            new_node.parent = nearest
            new_node.cost = nearest.cost + math.sqrt(
                (new_node.x - nearest.x)**2 + (new_node.y - nearest.y)**2
            )
            self.nodes.append(new_node)
            return new_node
        
        return None
    
    def plan(self, iterations=RRT_MAX_ITERATIONS):
        """Ejecuta el algoritmo RRT por un número de iteraciones"""
        new_nodes = []
        for _ in range(iterations):
            node = self.extend_tree()
            if node:
                new_nodes.append(node)
        return new_nodes
    
    def get_best_exploration_goal(self):
        """Selecciona el mejor nodo para explorar (más alejado y seguro)"""
        if len(self.nodes) <= 1:
            return None
        
        # Evaluar nodos por distancia y dirección frontal
        best_node = None
        best_score = -float('inf')
        
        for node in self.nodes[1:]:  # Saltar el origen
            distance = math.sqrt(node.x**2 + node.y**2)
            angle = math.degrees(math.atan2(node.y, node.x))
            
            # Preferir nodos adelante
            forward_bonus = 1.0 if -60 <= angle <= 60 else 0.5
            
            # Preferir nodos más alejados pero no demasiado lejos
            if distance > 1500:
                distance_score = 1500 - (distance - 1500) * 0.5
            else:
                distance_score = distance
            
            score = distance_score * forward_bonus
            
            if score > best_score:
                best_score = score
                best_node = node
        
        return best_node
    
    def get_path_to_node(self, target_node):
        """Obtiene el camino desde el origen hasta el nodo objetivo"""
        path = []
        current = target_node
        
        while current is not None:
            path.append((current.x, current.y))
            current = current.parent
        
        path.reverse()
        return path

# ============================================================================
# FUNCIONES DE CONTROL DE MOTORES
# ============================================================================

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

def mover_hacia_punto(target_x, target_y, velocidad_base=120):
    """Mueve el robot hacia un punto objetivo usando control proporcional"""
    # Calcular distancia y ángulo al objetivo
    distance = math.sqrt(target_x**2 + target_y**2)
    angle_rad = math.atan2(target_y, target_x)
    
    # Control proporcional
    vx = velocidad_base * math.cos(angle_rad)
    vy = velocidad_base * math.sin(angle_rad)
    
    # Corrección angular
    omega = angle_rad * 30  # Ganancia proporcional para rotación
    
    enviar_pwm(vx, vy, omega)
    
    return distance

# ============================================================================
# FUNCIONES AUXILIARES
# ============================================================================

def cerrar_todo(signal_received=None, frame=None):
    """Limpieza al cerrar el programa"""
    global running
    running = False
    detener_motores()
    print("\nPrograma terminado")
    sys.exit(0)

signal.signal(signal.SIGINT, cerrar_todo)
signal.signal(signal.SIGTERM, cerrar_todo)

def polar_to_cartesian(angle_deg, distance_mm):
    """Convierte coordenadas polares a cartesianas para visualización"""
    angle_rad = math.radians(angle_deg)
    r = distance_mm * SCALE
    x = CENTER[0] + int(r * math.cos(angle_rad))
    y = CENTER[1] - int(r * math.sin(angle_rad))
    return x, y

def world_to_screen(x_mm, y_mm):
    """Convierte coordenadas del mundo a coordenadas de pantalla"""
    x_screen = CENTER[0] + int(x_mm * SCALE)
    y_screen = CENTER[1] - int(y_mm * SCALE)
    return x_screen, y_screen

# ============================================================================
# FUNCIÓN PRINCIPAL
# ============================================================================

def main():
    """Función principal del sistema autónomo con RRT"""
    global running
    
    # Inicializar pygame
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Navegación Autónoma con RRT + LIDAR")
    clock = pygame.time.Clock()
    font_small = pygame.font.SysFont(None, 22)
    font_title = pygame.font.SysFont(None, 30, bold=True)
    
    # Inicializar LIDAR
    print("Conectando al LIDAR...")
    lidar = PyRPlidar()
    lidar.connect(port="/dev/ttyUSB0", baudrate=460800, timeout=3)
    lidar.set_motor_pwm(500)
    time.sleep(2)
    scan_generator = lidar.force_scan()
    print("LIDAR conectado y funcionando")
    
    # Inicializar sistemas
    occupancy_grid = OccupancyGrid(size_mm=3000, resolution=50)
    rrt_planner = RRTPlanner(occupancy_grid)
    
    # Variables de control
    estado = "explorando"  # Estados: explorando, navegando, detenido
    objetivo_actual = None
    camino_actual = []
    indice_camino = 0
    tiempo_ultimo_plan = time.time()
    INTERVALO_REPLANIFICACION = 3.0  # segundos
    
    print("\n" + "="*60)
    print("🤖 SISTEMA DE NAVEGACIÓN AUTÓNOMA CON RRT INICIADO")
    print("="*60)
    print("📡 El robot explorará el entorno de forma autónoma")
    print("🗺️  Presiona Ctrl+C para detener")
    print("="*60 + "\n")
    
    try:
        points = []
        prev_angle = None
        frame_count = 0
        
        for scan in scan_generator():
            # Eventos de pygame
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:
                        # Pausar/reanudar
                        if estado != "detenido":
                            estado = "detenido"
                            detener_motores()
                            print("⏸️  Sistema pausado")
                        else:
                            estado = "explorando"
                            print("▶️  Sistema reanudado")
                    elif event.key == pygame.K_r:
                        # Resetear árbol RRT
                        rrt_planner.reset()
                        objetivo_actual = None
                        camino_actual = []
                        print("🔄 Árbol RRT reiniciado")
            
            if not running:
                break
            
            # Recolectar puntos del LIDAR
            if MIN_DISTANCE <= scan.distance <= MAX_DISTANCE:
                points.append((scan.angle, scan.distance))
            
            # Detectar completación de barrido
            if prev_angle is not None and scan.angle < prev_angle:
                frame_count += 1
                
                # Actualizar mapa de ocupación
                occupancy_grid.update_from_lidar(points)
                
                # Lógica de planificación y navegación
                if estado == "explorando":
                    # Replanificar periódicamente
                    if time.time() - tiempo_ultimo_plan > INTERVALO_REPLANIFICACION:
                        print("\n🌳 Expandiendo árbol RRT...")
                        
                        # Expandir el árbol RRT
                        nuevos_nodos = rrt_planner.plan(iterations=RRT_MAX_ITERATIONS)
                        
                        # Seleccionar objetivo de exploración
                        objetivo_actual = rrt_planner.get_best_exploration_goal()
                        
                        if objetivo_actual:
                            camino_actual = rrt_planner.get_path_to_node(objetivo_actual)
                            indice_camino = 1  # Empezar desde el segundo punto (el primero es el origen)
                            estado = "navegando"
                            
                            dist_objetivo = math.sqrt(objetivo_actual.x**2 + objetivo_actual.y**2)
                            print(f"🎯 Nuevo objetivo: ({objetivo_actual.x:.0f}, {objetivo_actual.y:.0f}) mm")
                            print(f"   Distancia: {dist_objetivo:.0f} mm")
                            print(f"   Nodos en camino: {len(camino_actual)}")
                            print(f"   Total de nodos en árbol: {len(rrt_planner.nodes)}")
                        else:
                            print("⚠️  No se encontró objetivo válido, replanificando...")
                        
                        tiempo_ultimo_plan = time.time()
                
                elif estado == "navegando":
                    # Seguir el camino planificado
                    if indice_camino < len(camino_actual):
                        target_x, target_y = camino_actual[indice_camino]
                        
                        # Mover hacia el siguiente waypoint
                        distancia = mover_hacia_punto(target_x, target_y, velocidad_base=120)
                        
                        # Si llegamos cerca del waypoint, avanzar al siguiente
                        if distancia < 200:  # 20 cm de tolerancia
                            indice_camino += 1
                            if indice_camino >= len(camino_actual):
                                print("✅ Objetivo alcanzado")
                                estado = "explorando"
                                objetivo_actual = None
                                detener_motores()
                                time.sleep(0.5)
                    else:
                        # Camino completado
                        estado = "explorando"
                        objetivo_actual = None
                        detener_motores()
                
                elif estado == "detenido":
                    detener_motores()
                
                # ============================================================
                # VISUALIZACIÓN
                # ============================================================
                screen.fill(BLACK)
                
                # Dibujar círculos de referencia
                for r in range(500, MAX_DISTANCE + 1, 500):
                    pygame.draw.circle(screen, DARK_GREEN, CENTER, int(r * SCALE), 1)
                    label = font_small.render(f"{r//10} cm", True, GRAY)
                    screen.blit(label, (CENTER[0] + int(r * SCALE) - 30, CENTER[1] - 10))
                
                # Dibujar mapa de ocupación (opcional, puede ser costoso)
                # Se puede omitir para mejor rendimiento
                
                # Dibujar árbol RRT
                for node in rrt_planner.nodes:
                    if node.parent:
                        x1, y1 = world_to_screen(node.parent.x, node.parent.y)
                        x2, y2 = world_to_screen(node.x, node.y)
                        pygame.draw.line(screen, BLUE, (x1, y1), (x2, y2), 1)
                
                # Dibujar nodos del árbol
                for node in rrt_planner.nodes:
                    x, y = world_to_screen(node.x, node.y)
                    pygame.draw.circle(screen, CYAN, (x, y), 3)
                
                # Dibujar camino actual
                if camino_actual:
                    for i in range(len(camino_actual) - 1):
                        x1, y1 = world_to_screen(camino_actual[i][0], camino_actual[i][1])
                        x2, y2 = world_to_screen(camino_actual[i+1][0], camino_actual[i+1][1])
                        pygame.draw.line(screen, MAGENTA, (x1, y1), (x2, y2), 3)
                
                # Dibujar objetivo actual
                if objetivo_actual:
                    x, y = world_to_screen(objetivo_actual.x, objetivo_actual.y)
                    pygame.draw.circle(screen, YELLOW, (x, y), 8)
                    pygame.draw.circle(screen, YELLOW, (x, y), 12, 2)
                
                # Dibujar puntos LIDAR
                for ang, dist in points:
                    px, py = polar_to_cartesian(ang, dist)
                    
                    # Colorear según distancia
                    if dist <= OBSTACLE_THRESHOLD:
                        color = RED
                    elif dist <= 1500:
                        color = YELLOW
                    else:
                        color = GREEN
                    
                    pygame.draw.circle(screen, color, (px, py), 2)
                
                # Dibujar robot en el centro
                pygame.draw.circle(screen, WHITE, CENTER, 8)
                pygame.draw.circle(screen, YELLOW, CENTER, 5)
                
                # Indicador de dirección del robot
                pygame.draw.line(screen, WHITE, CENTER, 
                               (CENTER[0] + 20, CENTER[1]), 3)
                
                # Información en pantalla
                info_y = 10
                info_x = 10
                
                # Título
                title = font_title.render("🤖 Navegación Autónoma con RRT", True, CYAN)
                screen.blit(title, (WIDTH // 2 - title.get_width() // 2, info_y))
                info_y += 35
                
                # Estado
                estado_color = GREEN if estado == "navegando" else YELLOW if estado == "explorando" else RED
                estado_text = font_small.render(f"Estado: {estado.upper()}", True, estado_color)
                screen.blit(estado_text, (info_x, info_y))
                info_y += 25
                
                # Estadísticas del árbol
                stats_text = font_small.render(f"Nodos en árbol: {len(rrt_planner.nodes)}", True, WHITE)
                screen.blit(stats_text, (info_x, info_y))
                info_y += 20
                
                # Objetivo actual
                if objetivo_actual:
                    dist_obj = math.sqrt(objetivo_actual.x**2 + objetivo_actual.y**2)
                    obj_text = font_small.render(f"Objetivo: {dist_obj:.0f} mm", True, YELLOW)
                    screen.blit(obj_text, (info_x, info_y))
                    info_y += 20
                
                # Waypoint actual
                if camino_actual and indice_camino < len(camino_actual):
                    wp_text = font_small.render(f"Waypoint: {indice_camino}/{len(camino_actual)}", True, MAGENTA)
                    screen.blit(wp_text, (info_x, info_y))
                    info_y += 20
                
                # Controles
                info_y = HEIGHT - 70
                controls_text = font_small.render("ESPACIO: Pausar/Reanudar | R: Resetear árbol", True, GRAY)
                screen.blit(controls_text, (WIDTH // 2 - controls_text.get_width() // 2, info_y))
                
                pygame.display.flip()
                clock.tick(60)
                points.clear()
            
            prev_angle = scan.angle
    
    except KeyboardInterrupt:
        print("\n⚠️  Interrumpido por usuario")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n🛑 Cerrando sistema...")
        detener_motores()
        lidar.stop()
        lidar.set_motor_pwm(0)
        lidar.disconnect()
        pygame.quit()
        print("✅ Sistema cerrado correctamente")

if __name__ == "__main__":
    main()
