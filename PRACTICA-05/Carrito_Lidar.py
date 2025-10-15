# -*- coding: utf-8 -*-
# ============================================
# CÓDIGO PARA RASPBERRY PI CON LIDAR
# ============================================
# Este código corre en la Raspberry Pi y:
# 1. Lee datos del sensor LIDAR
# 2. Detecta obstáculos
# 3. Envía alertas a Firebase
# 4. Controla los motores del robot
# ============================================

import time
import math
import numpy as np
import signal
import sys
import json
from collections import deque

# Importar LIDAR
try:
    from pyrplidar import PyRPlidar
    LIDAR_AVAILABLE = True
except ImportError:
    print("[ERROR] pyrplidar no disponible. Instalar: pip install pyrplidar")
    sys.exit(1)

# Importar Firebase
try:
    from firebase_admin import credentials, db, initialize_app
    FIREBASE_AVAILABLE = True
except ImportError:
    print("[ERROR] firebase-admin no disponible. Instalar: pip install firebase-admin")
    sys.exit(1)

# Importar I2C para motores
try:
    import smbus2 as smbus
    bus = smbus.SMBus(1)
    bus.write_quick(0x34)
    print("[OK] I2C conectado")
    I2C_AVAILABLE = True
except Exception as e:
    print(f"[WARN] I2C no disponible: {e}")
    I2C_AVAILABLE = False
    class FakeBus:
        def write_i2c_block_data(self, addr, reg, data):
            print(f"[SIM] Motor -> {data}")
    bus = FakeBus()

# ==========================
# CONFIGURACIÓN
# ==========================
CONFIG_FILE = "lidar_config.json"

# LIDAR
LIDAR_CONFIG = {
    'port': '/dev/ttyUSB0',
    'baudrate': 460800,
    'timeout': 3,
    'motor_pwm': 500,
    'min_distance': 50,           # mm
    'max_distance': 3000,         # mm
    'obstacle_threshold': 800,    # mm (80 cm) - AJUSTABLE
    'front_angle_range': 60,      # grados cada lado del frente
    'side_angle_inner': 60,       # inicio zona lateral
    'side_angle_outer': 120,      # fin zona lateral
    'min_points_confirm': 5       # puntos mínimos para confirmar
}

# Motores
MOTOR_CONFIG = {
    'direccion': 0x34,
    'reg_velocidad': 0x33,
    'R': 0.048,           # radio rueda (m)
    'l1': 0.097,          # distancia centro-rueda eje X
    'l2': 0.109,          # distancia centro-rueda eje Y
    'v_max': 250,
    'pwm_max': 100
}

# Firebase
FIREBASE_CONFIG = {
    'cred_path': 'cred.json',
    'database_url': 'https://iot-app-f878d-default-rtdb.firebaseio.com/',
    'robot_path': 'robots/123456'
}

# Variables globales
running = True
lidar = None
firebase_ref = None
command_ref = None

# ==========================
# CONTROL DE MOTORES
# ==========================
def calcular_matriz_cinematica():
    """Calcula matriz cinemática del robot omnidireccional"""
    R = MOTOR_CONFIG['R']
    l1 = MOTOR_CONFIG['l1']
    l2 = MOTOR_CONFIG['l2']
    return (1 / R) * np.array([
        [1, -1, (l1 + l2)],
        [1, 1, (l1 + l2)],
        [1, 1, -(l1 + l2)],
        [1, -1, -(l1 + l2)]
    ])

W_MATRIX = calcular_matriz_cinematica()

def calcular_pwm(vx, vy, omega):
    """Calcula PWM para cada motor"""
    V = np.array([vx, vy, omega])
    velocidades = np.dot(W_MATRIX, V)
    
    # Escalar si excede límites
    max_vel = np.max(np.abs(velocidades))
    if max_vel > MOTOR_CONFIG['v_max']:
        velocidades = velocidades * (MOTOR_CONFIG['v_max'] / max_vel)
    
    # Invertir motores específicos
    velocidades[1] *= -1
    velocidades[2] *= -1
    
    # Convertir a PWM
    pwm = np.clip(
        (velocidades / MOTOR_CONFIG['v_max']) * MOTOR_CONFIG['pwm_max'],
        -MOTOR_CONFIG['pwm_max'],
        MOTOR_CONFIG['pwm_max']
    )
    
    return [int(p) for p in pwm]

def enviar_pwm(vx, vy, omega):
    """Envía comandos PWM a los motores"""
    if not I2C_AVAILABLE:
        return
    try:
        pwm = calcular_pwm(vx, vy, omega)
        bus.write_i2c_block_data(
            MOTOR_CONFIG['direccion'],
            MOTOR_CONFIG['reg_velocidad'],
            pwm
        )
    except Exception as e:
        print(f"[ERROR] Motor: {e}")

def detener_motores():
    """Detiene todos los motores"""
    if not I2C_AVAILABLE:
        return
    try:
        bus.write_i2c_block_data(
            MOTOR_CONFIG['direccion'],
            MOTOR_CONFIG['reg_velocidad'],
            [0, 0, 0, 0]
        )
        print("[MOTOR] Detenido")
    except Exception as e:
        print(f"[ERROR] Detener motor: {e}")

# ==========================
# FIREBASE
# ==========================
def conectar_firebase():
    """Conecta a Firebase"""
    global firebase_ref, command_ref
    
    try:
        cred = credentials.Certificate(FIREBASE_CONFIG['cred_path'])
        initialize_app(cred, {
            'databaseURL': FIREBASE_CONFIG['database_url']
        })
        
        base_path = FIREBASE_CONFIG['robot_path']
        firebase_ref = db.reference(f"{base_path}/lidar")
        command_ref = db.reference(f"{base_path}/instrucciones")
        
        # Inicializar estructura
        firebase_ref.set({
            'conectado': True,
            'obstaculo_detectado': False,
            'distancia_minima': LIDAR_CONFIG['max_distance'],
            'direccion_libre': 'none',
            'timestamp': time.time()
        })
        
        print("[OK] Firebase conectado")
        return True
        
    except Exception as e:
        print(f"[ERROR] Firebase: {e}")
        return False

def actualizar_firebase(obstaculo, distancia, direccion):
    """Actualiza estado del LIDAR en Firebase"""
    if firebase_ref is None:
        return
    
    try:
        firebase_ref.update({
            'obstaculo_detectado': obstaculo,
            'distancia_minima': int(distancia),
            'direccion_libre': direccion if direccion else 'none',
            'timestamp': time.time()
        })
    except Exception as e:
        print(f"[ERROR] Actualizar Firebase: {e}")

def leer_comandos_firebase():
    """Lee comandos de movimiento desde Firebase"""
    if command_ref is None:
        return None, None, None
    
    try:
        mov = command_ref.child("movimiento").get()
        rot = command_ref.child("rotación").get()
        parar = command_ref.child("parar").get()
        
        if parar:
            return 0, 0, 0
        
        vx = int(mov.get('vx', 0)) if mov else 0
        vy = int(mov.get('vy', 0)) if mov else 0
        w = int(rot.get('w', 0)) if rot else 0
        
        return vx, vy, w
        
    except Exception as e:
        print(f"[ERROR] Leer comandos: {e}")
        return None, None, None

# ==========================
# PROCESAMIENTO LIDAR
# ==========================
def detectar_obstaculo_frontal(points):
    """
    Analiza puntos del LIDAR y detecta obstáculos
    Retorna: (hay_obstaculo, distancia_minima, direccion_libre)
    """
    threshold = LIDAR_CONFIG['obstacle_threshold']
    front_range = LIDAR_CONFIG['front_angle_range']
    side_inner = LIDAR_CONFIG['side_angle_inner']
    side_outer = LIDAR_CONFIG['side_angle_outer']
    
    obstaculos_frente = []
    obstaculos_izq = []
    obstaculos_der = []
    
    for ang, dist in points:
        ang = ang % 360
        
        # Zona frontal: -front_range a +front_range
        if (ang >= 0 and ang <= front_range) or (ang >= 360 - front_range):
            if dist <= threshold:
                obstaculos_frente.append(dist)
        
        # Zona izquierda
        elif ang >= side_inner and ang <= side_outer:
            if dist <= threshold:
                obstaculos_izq.append(dist)
        
        # Zona derecha
        elif ang >= 360 - side_outer and ang <= 360 - side_inner:
            if dist <= threshold:
                obstaculos_der.append(dist)
    
    # Confirmar obstáculo (múltiples puntos)
    hay_obstaculo = len(obstaculos_frente) >= LIDAR_CONFIG['min_points_confirm']
    distancia_min = min(obstaculos_frente) if obstaculos_frente else LIDAR_CONFIG['max_distance']
    
    # Determinar dirección más libre
    direccion_libre = None
    if hay_obstaculo:
        dist_izq = min(obstaculos_izq) if obstaculos_izq else LIDAR_CONFIG['max_distance']
        dist_der = min(obstaculos_der) if obstaculos_der else LIDAR_CONFIG['max_distance']
        direccion_libre = 'left' if dist_izq > dist_der else 'right'
    
    return hay_obstaculo, distancia_min, direccion_libre

# ==========================
# INICIALIZACIÓN LIDAR
# ==========================
def inicializar_lidar():
    """Inicializa el sensor LIDAR"""
    global lidar
    
    try:
        lidar = PyRPlidar()
        lidar.connect(
            port=LIDAR_CONFIG['port'],
            baudrate=LIDAR_CONFIG['baudrate'],
            timeout=LIDAR_CONFIG['timeout']
        )
        lidar.set_motor_pwm(LIDAR_CONFIG['motor_pwm'])
        time.sleep(2)
        print("[OK] LIDAR inicializado")
        return True
    except Exception as e:
        print(f"[ERROR] LIDAR: {e}")
        return False

def cerrar_lidar():
    """Cierra conexión del LIDAR"""
    global lidar
    if lidar:
        try:
            lidar.stop()
            lidar.set_motor_pwm(0)
            lidar.disconnect()
            print("[OK] LIDAR cerrado")
        except:
            pass

# ==========================
# MANEJO DE SEÑALES
# ==========================
def cerrar_todo(signal_received=None, frame=None):
    """Limpieza al cerrar"""
    global running
    running = False
    print("\n[INFO] Cerrando sistema...")
    detener_motores()
    cerrar_lidar()
    if firebase_ref:
        firebase_ref.update({'conectado': False})
    sys.exit(0)

signal.signal(signal.SIGINT, cerrar_todo)
signal.signal(signal.SIGTERM, cerrar_todo)

# ==========================
# GUARDADO/CARGA CONFIG
# ==========================
def guardar_config():
    """Guarda configuración"""
    config = {
        'lidar': LIDAR_CONFIG,
        'motor': MOTOR_CONFIG,
        'firebase': FIREBASE_CONFIG
    }
    try:
        with open(CONFIG_FILE, 'w') as f:
            json.dump(config, f, indent=4)
        print(f"[OK] Configuración guardada en {CONFIG_FILE}")
    except Exception as e:
        print(f"[ERROR] Guardar config: {e}")

def cargar_config():
    """Carga configuración"""
    global LIDAR_CONFIG, MOTOR_CONFIG, FIREBASE_CONFIG
    
    if not os.path.exists(CONFIG_FILE):
        return
    
    try:
        with open(CONFIG_FILE, 'r') as f:
            config = json.load(f)
        
        if 'lidar' in config:
            LIDAR_CONFIG.update(config['lidar'])
        if 'motor' in config:
            MOTOR_CONFIG.update(config['motor'])
        if 'firebase' in config:
            FIREBASE_CONFIG.update(config['firebase'])
        
        print(f"[OK] Configuración cargada")
    except Exception as e:
        print(f"[ERROR] Cargar config: {e}")

# ==========================
# FUNCIÓN PRINCIPAL
# ==========================
def main():
    """Función principal del sistema LIDAR"""
    global running
    
    print("=" * 50)
    print("SISTEMA DE DETECCIÓN LIDAR")
    print("Raspberry Pi + LIDAR + Firebase")
    print("=" * 50)
    
    # Cargar configuración
    cargar_config()
    
    # Conectar Firebase
    if not conectar_firebase():
        print("[ERROR] No se pudo conectar a Firebase")
        return
    
    # Inicializar LIDAR
    if not inicializar_lidar():
        print("[ERROR] No se pudo inicializar LIDAR")
        return
    
    print("\n[INFO] Sistema iniciado. Presiona Ctrl+C para detener.\n")
    
    scan_generator = lidar.force_scan()
    points = []
    prev_angle = None
    
    ultimo_estado = False
    contador_fps = 0
    tiempo_fps = time.time()
    
    try:
        for scan in scan_generator():
            if not running:
                break
            
            # Recolectar puntos
            if LIDAR_CONFIG['min_distance'] <= scan.distance <= LIDAR_CONFIG['max_distance']:
                points.append((scan.angle, scan.distance))
            
            # Detectar completación de barrido (360°)
            if prev_angle is not None and scan.angle < prev_angle:
                contador_fps += 1
                
                # Analizar obstáculos
                hay_obstaculo, dist_min, direccion = detectar_obstaculo_frontal(points)
                
                # Actualizar Firebase
                actualizar_firebase(hay_obstaculo, dist_min, direccion)
                
                # Leer comandos desde Firebase
                vx, vy, w = leer_comandos_firebase()
                if vx is not None:
                    enviar_pwm(vx, vy, w)
                
                # Log de cambios de estado
                if hay_obstaculo != ultimo_estado:
                    if hay_obstaculo:
                        print(f"[ALERTA] Obstáculo detectado: {int(dist_min)}mm - Girar {direccion}")
                    else:
                        print(f"[OK] Camino libre")
                    ultimo_estado = hay_obstaculo
                
                # Mostrar FPS cada segundo
                if time.time() - tiempo_fps >= 1.0:
                    print(f"[INFO] LIDAR: {contador_fps} Hz | Dist: {int(dist_min)}mm | Obst: {hay_obstaculo}")
                    contador_fps = 0
                    tiempo_fps = time.time()
                
                points.clear()
            
            prev_angle = scan.angle
    
    except KeyboardInterrupt:
        print("\n[INFO] Interrumpido por usuario")
    except Exception as e:
        print(f"[ERROR] Error en bucle principal: {e}")
    finally:
        cerrar_todo()

if __name__ == "__main__":
    import os
    main()