import time
import math
import numpy as np
import pygame
from pyrplidar import PyRPlidar
import signal
import sys

# Configuración del LIDAR
MIN_DISTANCE = 50      # mm (5 cm)
MAX_DISTANCE = 3000    # mm (300 cm)
OBSTACLE_THRESHOLD = 800  # mm (80 cm) - distancia para detectar obstáculos
FRONT_ANGLE_RANGE = 60  # grados a cada lado del frente (total 120°)

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

# Pantalla para visualización (opcional)
WIDTH, HEIGHT = 800, 800
CENTER = (WIDTH // 2, HEIGHT // 2)
SCALE = (WIDTH // 2) / MAX_DISTANCE
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
YELLOW = (255, 255, 0)
RED = (255, 0, 0)
DARK_GREEN = (0, 100, 0)
WHITE = (255, 255, 255)

# Variables globales
running = True

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

def polar_to_cartesian(angle_deg, distance_mm):
    """Convierte coordenadas polares a cartesianas para visualización"""
    angle_rad = math.radians(angle_deg)
    r = distance_mm * SCALE
    x = CENTER[0] + int(r * math.cos(angle_rad))
    y = CENTER[1] - int(r * math.sin(angle_rad))
    return x, y

def detectar_obstaculo_frontal(points):
    """
    Analiza los puntos del LIDAR y detecta obstáculos en el frente
    Retorna: (hay_obstaculo, distancia_minima, direccion_libre)
    direccion_libre: 'left', 'right', o None
    """
    obstaculos_frente = []
    obstaculos_izq = []
    obstaculos_der = []
    
    for ang, dist in points:
        # Normalizar ángulo (0-360)
        ang = ang % 360
        
        # Frente: -60° a 60° (o 300° a 360° y 0° a 60°)
        if (ang >= 0 and ang <= FRONT_ANGLE_RANGE) or (ang >= 360 - FRONT_ANGLE_RANGE):
            if dist <= OBSTACLE_THRESHOLD:
                obstaculos_frente.append(dist)
                
        # Izquierda: 60° a 120°
        elif ang >= FRONT_ANGLE_RANGE and ang <= 120:
            if dist <= OBSTACLE_THRESHOLD:
                obstaculos_izq.append(dist)
                
        # Derecha: 240° a 300°
        elif ang >= 240 and ang <= 360 - FRONT_ANGLE_RANGE:
            if dist <= OBSTACLE_THRESHOLD:
                obstaculos_der.append(dist)
    
    hay_obstaculo = len(obstaculos_frente) > 5  # Requerir varios puntos para confirmar
    distancia_min = min(obstaculos_frente) if obstaculos_frente else MAX_DISTANCE
    
    # Determinar dirección más libre
    if hay_obstaculo:
        dist_izq = min(obstaculos_izq) if obstaculos_izq else MAX_DISTANCE
        dist_der = min(obstaculos_der) if obstaculos_der else MAX_DISTANCE
        direccion_libre = 'left' if dist_izq > dist_der else 'right'
    else:
        direccion_libre = None
    
    return hay_obstaculo, distancia_min, direccion_libre

def main():
    """Función principal del sistema autónomo"""
    global running
    
    # Inicializar pygame para visualización
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Sistema Autónomo con LIDAR")
    clock = pygame.time.Clock()
    font_small = pygame.font.SysFont(None, 20)
    font_title = pygame.font.SysFont(None, 28, bold=True)
    
    # Inicializar LIDAR
    lidar = PyRPlidar()
    lidar.connect(port="/dev/ttyUSB0", baudrate=460800, timeout=3)
    lidar.set_motor_pwm(500)
    time.sleep(2)
    scan_generator = lidar.force_scan()
    
    print("Sistema autónomo iniciado. Presiona Ctrl+C para detener.")
    
    # Variables de control
    estado = "avanzando"  # Estados: avanzando, girando, detenido
    tiempo_giro = 0
    velocidad_avance = 100  # mm/s
    velocidad_giro = 50     # grados/s
    
    try:
        points = []
        prev_angle = None
        
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
                # Analizar obstáculos
                hay_obstaculo, dist_min, direccion = detectar_obstaculo_frontal(points)
                
                # Lógica de control
                if estado == "avanzando":
                    if hay_obstaculo:
                        print(f"¡Obstáculo detectado a {dist_min}mm! Girando a la {direccion}")
                        detener_motores()
                        estado = "girando"
                        tiempo_giro = time.time()
                        # Guardar dirección de giro
                        if direccion == 'left':
                            enviar_pwm(0, 0, velocidad_giro)  # Giro a la izquierda
                        else:
                            enviar_pwm(0, 0, -velocidad_giro)  # Giro a la derecha
                    else:
                        # Continuar avanzando
                        enviar_pwm(velocidad_avance, 0, 0)
                
                elif estado == "girando":
                    # Girar por 1.5 segundos
                    if time.time() - tiempo_giro > 1.5:
                        print("Giro completado, reanudando avance")
                        estado = "avanzando"
                        enviar_pwm(velocidad_avance, 0, 0)
                
                # Visualización
                screen.fill(BLACK)
                
                # Dibujar círculos de referencia
                for r in range(500, MAX_DISTANCE + 1, 500):
                    pygame.draw.circle(screen, DARK_GREEN, CENTER, int(r * SCALE), 1)
                    label = font_small.render(f"{r//10} cm", True, WHITE)
                    screen.blit(label, (CENTER[0] + int(r * SCALE) - 25, CENTER[1]))
                
                # Dibujar puntos LIDAR con colores según distancia
                for ang, dist in points:
                    px, py = polar_to_cartesian(ang, dist)
                    
                    # Colorear según zona
                    ang_norm = ang % 360
                    if (ang_norm <= FRONT_ANGLE_RANGE or ang_norm >= 360 - FRONT_ANGLE_RANGE):
                        # Zona frontal
                        if dist <= OBSTACLE_THRESHOLD:
                            color = RED
                        else:
                            color = GREEN
                    else:
                        # Otras zonas
                        if dist <= 1000:
                            color = RED
                        elif dist <= 2000:
                            color = YELLOW
                        else:
                            color = GREEN
                    
                    pygame.draw.circle(screen, color, (px, py), 2)
                
                # Indicador de zona frontal
                pygame.draw.arc(screen, WHITE, 
                               (CENTER[0] - OBSTACLE_THRESHOLD * SCALE,
                                CENTER[1] - OBSTACLE_THRESHOLD * SCALE,
                                OBSTACLE_THRESHOLD * SCALE * 2,
                                OBSTACLE_THRESHOLD * SCALE * 2),
                               math.radians(90 - FRONT_ANGLE_RANGE),
                               math.radians(90 + FRONT_ANGLE_RANGE), 2)
                
                # Información en pantalla
                status_text = f"Estado: {estado.upper()}"
                if hay_obstaculo:
                    status_text += f" | Obstáculo: {int(dist_min)}mm"
                
                status_surface = font_title.render(status_text, True, YELLOW if hay_obstaculo else GREEN)
                screen.blit(status_surface, (WIDTH // 2 - status_surface.get_width() // 2, 20))
                
                title_surface = font_title.render("Sistema Autónomo con LIDAR", True, WHITE)
                screen.blit(title_surface, (WIDTH // 2 - title_surface.get_width() // 2, HEIGHT - 40))
                
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

if _name_ == "_main_":
    main()