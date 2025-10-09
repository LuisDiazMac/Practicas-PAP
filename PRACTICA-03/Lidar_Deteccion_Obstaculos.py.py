import math
import pygame
from pyrplidar import PyRPlidar

# --- Parámetros del sistema ---
WIDTH, HEIGHT = 800, 800
CENTER = (WIDTH // 2, HEIGHT // 2)
MAX_DISTANCE = 6000  # mm
SCALE = (WIDTH // 2) / MAX_DISTANCE
DISTANCIA_FRENADO = 50  # mm = 5 cm

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Detección de Obstáculos con LiDAR")
clock = pygame.time.Clock()

lidar = PyRPlidar()
lidar.connect(port="/dev/ttyUSB0", baudrate=115200, timeout=3)
print("[INFO] LiDAR conectado correctamente")

scan_generator = lidar.start_scan_express(ExpressScanType.NORMAL)
running = True

while running:
    screen.fill((0, 0, 0))

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    obstaculo_detectado = False

    for scan in scan_generator():
        for _, angle, distance in scan:
            if distance > 0:
                x = CENTER[0] + int(distance * SCALE * math.cos(math.radians(angle)))
                y = CENTER[1] + int(distance * SCALE * math.sin(math.radians(angle)))
                color = (0, 255, 0)

                if distance <= DISTANCIA_FRENADO:
                    color = (255, 0, 0)
                    obstaculo_detectado = True

                pygame.draw.circle(screen, color, (x, y), 2)
        break

    if obstaculo_detectado:
        print("[ALERTA] Obstáculo detectado a menos de 5 cm - ORDEN DE FRENADO")

    pygame.display.flip()
    clock.tick(60)

lidar.stop()
lidar.disconnect()
pygame.quit()
print("[OK] Ejecución finalizada correctamente.")
