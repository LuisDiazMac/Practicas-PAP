# -*- coding: utf-8 -*-
import cv2
import numpy as np
import math
import time
import threading
from collections import deque
import tkinter as tk
from tkinter import ttk, messagebox
from firebase_admin import credentials as _cred, db as _db, initialize_app as _fb_init
import json
import os
from PIL import Image, ImageTk
import queue

# ==========================
#  Clase principal de la aplicación
# ==========================
class ArucoRobotGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Control Robot ArUco - Versión Mejorada")
        self.root.geometry("1200x800")
        self.root.configure(bg='#2b2b2b')
        
        # Variables del sistema
        self.cap = None
        self.running = False
        self.camera_thread = None
        self.firebase_thread = None
        self.config_file = "robot_config.json"
        
        # Cola para comandos Firebase (evitar bloqueos)
        self.firebase_queue = queue.Queue(maxsize=10)
        
        # Variables para la visualización en tkinter
        self.video_label = None
        self.current_frame = None
        self.video_width = 640
        self.video_height = 480
        
        # Variables de configuración
        self.camera_index = tk.IntVar(value=1)
        self.camera_width = tk.IntVar(value=1920)
        self.camera_height = tk.IntVar(value=1080)
        self.camera_fps = tk.IntVar(value=30)
        self.robot_id = tk.IntVar(value=4)
        self.corner_ids = [tk.IntVar(value=1), tk.IntVar(value=2), tk.IntVar(value=5), tk.IntVar(value=3)]
        self.grid_n = tk.IntVar(value=10)
        
        # Variables del control
        self.kp_lin = tk.DoubleVar(value=80.0)
        self.kp_w_face = tk.DoubleVar(value=120.0)
        self.kp_w_hold = tk.DoubleVar(value=120.0)
        self.cmd_max = tk.IntVar(value=250)
        self.w_max = tk.IntVar(value=250)
        self.pos_tolerance = tk.DoubleVar(value=0.20)
        
        # Variables de estado
        self.current_fps = tk.StringVar(value="0.0 Hz")
        self.robot_position = tk.StringVar(value="No detectado")
        self.target_position = tk.StringVar(value="Sin objetivo")
        self.connection_status = tk.StringVar(value="Desconectado")
        self.orient_mode = tk.BooleanVar(value=True)

        # Parámetros del controlador PI
        self.ki_lin = tk.DoubleVar(value=15.0)
        self.ki_w = tk.DoubleVar(value=20.0)
        self._integral_ex = 0.0
        self._integral_ey = 0.0
        self._integral_w = 0.0
        self.integral_max_lin = tk.DoubleVar(value=50.0)
        self.integral_max_w = tk.DoubleVar(value=30.0)
        self.camera_delay = tk.DoubleVar(value=0.15)
        self.prediction_horizon = tk.IntVar(value=3)
        self._command_history = deque(maxlen=20)
        self._position_history = deque(maxlen=20)
        self._timestamp_history = deque(maxlen=20)
        self._velocity_estimate = np.array([0.0, 0.0])
        self._angular_velocity_estimate = 0.0
        self._last_integral_time = time.time()
        self.integral_status = tk.StringVar(value="I_lin: 0.0, I_ang: 0.0")
        self.velocity_estimate_status = tk.StringVar(value="v_est: (0.0, 0.0)")
        
        # Firebase
        self.firebase_url = tk.StringVar(value="https://iot-app-f878d-default-rtdb.firebaseio.com/")
        self.firebase_path = tk.StringVar(value="robots/123456")
        self.cred_path = tk.StringVar(value="cred.json")
        self.instr_ref = None
        self.firebase_connected = False
        
        # Variables del procesador de video
        self._target_g = None
        self._hold_heading_rad = None
        self.CURRENT_H = None
        self.CURRENT_H_INV = None
        self._last_robot_px = None
        self.display_scale = 0.75
        
        # MEJORAS PARA DETECCIÓN ROBUSTA
        # ===================================
        # Variables de optimización ArUco MEJORADAS
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        # Parámetros mejorados para mejor detección
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 23
        self.aruco_params.adaptiveThreshWinSizeStep = 10
        self.aruco_params.adaptiveThreshConstant = 7
        self.aruco_params.minMarkerPerimeterRate = 0.03
        self.aruco_params.maxMarkerPerimeterRate = 4.0
        self.aruco_params.polygonalApproxAccuracyRate = 0.03
        self.aruco_params.minCornerDistanceRate = 0.05
        self.aruco_params.minDistanceToBorder = 3
        self.aruco_params.minMarkerDistanceRate = 0.05
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.aruco_params.cornerRefinementWinSize = 5
        self.aruco_params.cornerRefinementMaxIterations = 30
        self.aruco_params.cornerRefinementMinAccuracy = 0.1
        self.aruco_params.markerBorderBits = 1
        self.aruco_params.perspectiveRemovePixelPerCell = 8
        self.aruco_params.perspectiveRemoveIgnoredMarginPerCell = 0.13
        self.aruco_params.maxErroneousBitsInBorderRate = 0.35
        self.aruco_params.minOtsuStdDev = 5.0
        self.aruco_params.errorCorrectionRate = 0.6
        
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # SISTEMA DE PREDICCIÓN Y SEGUIMIENTO
        # ===================================
        self._robot_tracker = {
            'detected': False,
            'lost_frames': 0,
            'max_lost_frames': 15,  # Máximo frames perdidos antes de parar
            'last_position': None,
            'last_heading': None,
            'velocity_history': deque(maxlen=5),
            'position_history': deque(maxlen=10),
            'prediction_enabled': True
        }
        
        # Cache de homografía mejorado
        self._homography_cache = {
            "H": None, 
            "H_inv": None, 
            "timestamp": 0,
            "stability_counter": 0,
            "min_stability": 3  # Mínimo frames estables antes de usar
        }
        
        # Variables de optimización de rendimiento AJUSTADAS
        self._last_command_time = 0
        self._command_interval = 0.05   # 20 Hz para comandos (más rápido)
        self._last_gui_update_time = 0
        self._gui_update_interval = 0.1  # 10 Hz para actualizaciones de GUI
        self._current_robot_heading = 0.0
        self._robot_detected = False
        self._frame_skip_counter = 0
        self._detection_skip = 0  # Procesar detección cada frame
        
        # Control de calidad de imagen
        self._frame_quality_threshold = 30.0  # Umbral de nitidez
        self._last_good_frame = None
        
        # Cargar configuración
        self.load_config()
        
        # Crear interfaz
        self.create_interface()
        
        # Configurar eventos de cierre
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        
    def create_interface(self):
        """Crea la interfaz gráfica completa"""
        # Estilo
        style = ttk.Style()
        style.theme_use('alt')
        style.configure('Title.TLabel', font=('Arial', 12, 'bold'), background='#2b2b2b', foreground='white')
        style.configure('Header.TLabel', font=('Arial', 10, 'bold'), background='#2b2b2b', foreground='#4CAF50')
        style.configure('Status.TLabel', font=('Arial', 9), background='#2b2b2b', foreground='#FFC107')
        
        # Panel principal dividido
        main_paned = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main_paned.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Panel izquierdo - Controles
        left_frame = ttk.Frame(main_paned, relief=tk.RAISED, borderwidth=2)
        main_paned.add(left_frame, weight=1)
        
        # Panel derecho - Video y estado
        right_frame = ttk.Frame(main_paned, relief=tk.RAISED, borderwidth=2)
        main_paned.add(right_frame, weight=2)
        
        self.create_control_panel(left_frame)
        self.create_video_panel(right_frame)
        
    def create_control_panel(self, parent):
        """Crea el panel de controles"""
        # Título
        title_label = ttk.Label(parent, text="Control Robot ArUco Mejorado", style='Title.TLabel')
        title_label.pack(pady=(10, 20))
        
        # Crear notebook para pestañas
        notebook = ttk.Notebook(parent)
        notebook.pack(fill=tk.BOTH, expand=True, padx=10)
        
        # Pestaña 1: Configuración del Robot
        robot_frame = ttk.Frame(notebook)
        notebook.add(robot_frame, text="Robot")
        self.create_robot_config(robot_frame)
        
        # Pestaña 2: Configuración de Cámara
        camera_frame = ttk.Frame(notebook)
        notebook.add(camera_frame, text="Camara")
        self.create_camera_config(camera_frame)
        
        # Pestaña 3: Control y Parámetros
        control_frame = ttk.Frame(notebook)
        notebook.add(control_frame, text="Control")
        self.create_control_config(control_frame)
        
        # Pestaña 4: Firebase
        firebase_frame = ttk.Frame(notebook)
        notebook.add(firebase_frame, text="Firebase")
        self.create_firebase_config(firebase_frame)

        # Pestaña 5: Control Manual
        manual_frame = ttk.Frame(notebook)
        notebook.add(manual_frame, text="Manual")
        self.create_manual_control(manual_frame)
        
        # Pestaña 6: Configuraciones Avanzadas (NUEVA)
        advanced_frame = ttk.Frame(notebook)
        notebook.add(advanced_frame, text="Avanzado")
        self.create_advanced_config(advanced_frame)
        
        # Botones principales
        self.create_main_buttons(parent)

    
    def create_advanced_config(self, parent):
        """Configuraciones avanzadas para detección robusta"""
        # Configuración de seguimiento
        tracking_group = ttk.LabelFrame(parent, text="Seguimiento Robusto", padding=10)
        tracking_group.pack(fill=tk.X, padx=10, pady=5)
        
        self.max_lost_frames = tk.IntVar(value=15)
        self.prediction_enabled = tk.BooleanVar(value=True)
        self.quality_threshold = tk.DoubleVar(value=30.0)
        
        ttk.Label(tracking_group, text="Max Frames Perdidos:").grid(row=0, column=0, sticky=tk.W, pady=2)
        ttk.Spinbox(tracking_group, from_=5, to=50, textvariable=self.max_lost_frames, width=10).grid(row=0, column=1, padx=(10,0), pady=2)
        
        ttk.Checkbutton(tracking_group, text="Habilitar Predicción", 
                       variable=self.prediction_enabled).grid(row=1, column=0, columnspan=2, sticky=tk.W, pady=2)
        
        ttk.Label(tracking_group, text="Umbral de Calidad:").grid(row=2, column=0, sticky=tk.W, pady=2)
        ttk.Scale(tracking_group, from_=10.0, to=100.0, variable=self.quality_threshold, 
                orient=tk.HORIZONTAL, length=150).grid(row=2, column=1, padx=(10,0), pady=2)
        
        # Estado de seguimiento
        status_group = ttk.LabelFrame(parent, text="Estado de Seguimiento", padding=10)
        status_group.pack(fill=tk.X, padx=10, pady=5)
        
        self.tracking_status = tk.StringVar(value="Inactivo")
        self.frames_lost = tk.StringVar(value="0")
        self.prediction_status = tk.StringVar(value="Deshabilitado")
        
        ttk.Label(status_group, text="Estado:").grid(row=0, column=0, sticky=tk.W, pady=2)
        ttk.Label(status_group, textvariable=self.tracking_status, style='Status.TLabel').grid(row=0, column=1, sticky=tk.W, pady=2)
        
        ttk.Label(status_group, text="Frames Perdidos:").grid(row=1, column=0, sticky=tk.W, pady=2)
        ttk.Label(status_group, textvariable=self.frames_lost, style='Status.TLabel').grid(row=1, column=1, sticky=tk.W, pady=2)
        
        ttk.Label(status_group, text="Predicción:").grid(row=2, column=0, sticky=tk.W, pady=2)
        ttk.Label(status_group, textvariable=self.prediction_status, style='Status.TLabel').grid(row=2, column=1, sticky=tk.W, pady=2)
    
    def create_robot_config(self, parent):
        """Configuración del robot y marcadores ArUco"""
        # ID del Robot
        robot_group = ttk.LabelFrame(parent, text="Identificación del Robot", padding=10)
        robot_group.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(robot_group, text="ID del Robot ArUco:").grid(row=0, column=0, sticky=tk.W, pady=2)
        robot_spin = ttk.Spinbox(robot_group, from_=0, to=50, textvariable=self.robot_id, width=10)
        robot_spin.grid(row=0, column=1, padx=(10,0), pady=2)
        
        # IDs de las esquinas
        corner_group = ttk.LabelFrame(parent, text="IDs de Esquinas de la Mesa", padding=10)
        corner_group.pack(fill=tk.X, padx=10, pady=5)
        
        corner_labels = ["Esquina 1:", "Esquina 2:", "Esquina 3:", "Esquina 4:"]
        for i, label_text in enumerate(corner_labels):
            ttk.Label(corner_group, text=label_text).grid(row=i//2, column=(i%2)*2, sticky=tk.W, pady=2, padx=(0,5))
            spin = ttk.Spinbox(corner_group, from_=0, to=50, textvariable=self.corner_ids[i], width=8)
            spin.grid(row=i//2, column=(i%2)*2+1, padx=(5,10), pady=2)
        
        # Configuración de la malla
        grid_group = ttk.LabelFrame(parent, text="Configuración de Malla", padding=10)
        grid_group.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(grid_group, text="Tamaño de Malla (NxN):").grid(row=0, column=0, sticky=tk.W, pady=2)
        ttk.Spinbox(grid_group, from_=5, to=20, textvariable=self.grid_n, width=10).grid(row=0, column=1, padx=(10,0), pady=2)
        
    def create_camera_config(self, parent):
        """Configuración de la cámara"""
        # Configuración básica
        basic_group = ttk.LabelFrame(parent, text="Configuración Básica", padding=10)
        basic_group.pack(fill=tk.X, padx=10, pady=5)
        
        configs = [
            ("Índice de Cámara:", self.camera_index, 0, 5),
            ("Ancho (píxeles):", self.camera_width, 640, 1920),
            ("Alto (píxeles):", self.camera_height, 480, 1080),
            ("FPS Objetivo:", self.camera_fps, 15, 60)
        ]
        
        for i, (label_text, var, min_val, max_val) in enumerate(configs):
            ttk.Label(basic_group, text=label_text).grid(row=i, column=0, sticky=tk.W, pady=2)
            ttk.Spinbox(basic_group, from_=min_val, to=max_val, textvariable=var, width=10).grid(row=i, column=1, padx=(10,0), pady=2)
        
        
        # Resoluciones predefinidas
        res_group = ttk.LabelFrame(parent, text="Resoluciones Rápidas", padding=10)
        res_group.pack(fill=tk.X, padx=10, pady=5)
        
        res_buttons = [
            ("1080p", 1920, 1080),
            ("720p", 1280, 720),
            ("480p", 640, 480)
        ]
        
        for i, (name, w, h) in enumerate(res_buttons):
            btn = ttk.Button(res_group, text=name, 
                        command=lambda w=w, h=h: self.set_resolution(w, h))
            btn.grid(row=0, column=i, padx=5, pady=5)
        
        # Compensación de delay (MOVIDO DESDE CONTROL)
        delay_group = ttk.LabelFrame(parent, text="Compensación de Delay", padding=10)
        delay_group.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(delay_group, text="Delay Cámara (s):").grid(row=0, column=0, sticky=tk.W, pady=2)
        ttk.Spinbox(delay_group, from_=0.05, to=0.5, textvariable=self.camera_delay,
                width=12, increment=0.01, format="%.3f").grid(row=0, column=1, padx=(10,0), pady=2)
        
        ttk.Label(delay_group, text="Horizonte Predicción:").grid(row=1, column=0, sticky=tk.W, pady=2)
        ttk.Spinbox(delay_group, from_=1, to=10, textvariable=self.prediction_horizon,
                width=12, increment=1).grid(row=1, column=1, padx=(10,0), pady=2)
            
    def create_control_config(self, parent):
        """Configuración de parámetros de control"""
        # Ganancias del controlador P
        gains_group = ttk.LabelFrame(parent, text="Ganancias Proporcionales (Kp)", padding=10)
        gains_group.pack(fill=tk.X, padx=10, pady=5)
        
        gain_configs = [
            ("Kp Lineal:", self.kp_lin, 1.0, 200.0),
            ("Kp Angular (Movimiento):", self.kp_w_face, 1.0, 300.0),
            ("Kp Angular (Fijo):", self.kp_w_hold, 1.0, 300.0)
        ]
        
        for i, (label_text, var, min_val, max_val) in enumerate(gain_configs):
            ttk.Label(gains_group, text=label_text).grid(row=i, column=0, sticky=tk.W, pady=2)
            ttk.Spinbox(gains_group, from_=min_val, to=max_val, textvariable=var, 
                    width=12, increment=1.0, format="%.1f").grid(row=i, column=1, padx=(10,0), pady=2)
        
        # Ganancias Integrales (Ki)
        integral_group = ttk.LabelFrame(parent, text="Ganancias Integrales (Ki)", padding=10)
        integral_group.pack(fill=tk.X, padx=10, pady=5)
        
        # Ki Lineal
        ttk.Label(integral_group, text="Ki Lineal:").grid(row=0, column=0, sticky=tk.W, pady=2)
        ttk.Spinbox(integral_group, from_=0.0, to=50.0, textvariable=self.ki_lin,
                width=12, increment=0.5, format="%.1f").grid(row=0, column=1, padx=(10,0), pady=2)
        
        # Ki Angular
        ttk.Label(integral_group, text="Ki Angular:").grid(row=1, column=0, sticky=tk.W, pady=2)
        ttk.Spinbox(integral_group, from_=0.0, to=50.0, textvariable=self.ki_w,
                width=12, increment=0.5, format="%.1f").grid(row=1, column=1, padx=(10,0), pady=2)
        
        # Anti-windup
        ttk.Label(integral_group, text="Max Integral Lineal:").grid(row=2, column=0, sticky=tk.W, pady=2)
        ttk.Spinbox(integral_group, from_=10, to=200, textvariable=self.integral_max_lin, 
                width=12, increment=5).grid(row=2, column=1, padx=(10,0), pady=2)
        
        ttk.Label(integral_group, text="Max Integral Angular:").grid(row=3, column=0, sticky=tk.W, pady=2)
        ttk.Spinbox(integral_group, from_=10, to=100, textvariable=self.integral_max_w,
                width=12, increment=5).grid(row=3, column=1, padx=(10,0), pady=2)
        
        # Botón reset integradores
        reset_btn = ttk.Button(integral_group, text="Reset Integradores", 
                            command=self.reset_integral_accumulators)
        reset_btn.grid(row=4, column=0, columnspan=2, pady=5)
        
        # Límites y tolerancias
        limits_group = ttk.LabelFrame(parent, text="Límites y Tolerancias", padding=10)
        limits_group.pack(fill=tk.X, padx=10, pady=5)
        
        limit_configs = [
            ("Vel. Max Lineal:", self.cmd_max, 50, 500, 10),
            ("Vel. Max Angular:", self.w_max, 50, 500, 10),
            ("Tolerancia Posición:", self.pos_tolerance, 0.1, 1.0, 0.05)
        ]
        
        for i, config in enumerate(limit_configs):
            label_text, var = config[:2]
            ttk.Label(limits_group, text=label_text).grid(row=i, column=0, sticky=tk.W, pady=2)
            if len(config) > 4:  # Es float
                min_val, max_val, increment = config[2:]
                ttk.Spinbox(limits_group, from_=min_val, to=max_val, textvariable=var, 
                        width=12, increment=increment, format="%.2f").grid(row=i, column=1, padx=(10,0), pady=2)
            else:  # Es int
                min_val, max_val, increment = config[2:5]
                ttk.Spinbox(limits_group, from_=min_val, to=max_val, textvariable=var, 
                        width=12, increment=increment).grid(row=i, column=1, padx=(10,0), pady=2)

    def create_firebase_config(self, parent):
        """Configuración de Firebase"""
        # Configuración de conexión
        conn_group = ttk.LabelFrame(parent, text="Configuración de Conexión", padding=10)
        conn_group.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(conn_group, text="URL de Firebase:").grid(row=0, column=0, sticky=tk.W, pady=2)
        url_entry = ttk.Entry(conn_group, textvariable=self.firebase_url, width=30)
        url_entry.grid(row=1, column=0, columnspan=2, sticky=tk.EW, pady=2)
        
        ttk.Label(conn_group, text="Ruta del Robot:").grid(row=2, column=0, sticky=tk.W, pady=2)
        path_entry = ttk.Entry(conn_group, textvariable=self.firebase_path, width=30)
        path_entry.grid(row=3, column=0, columnspan=2, sticky=tk.EW, pady=2)
        
        ttk.Label(conn_group, text="Archivo de Credenciales:").grid(row=4, column=0, sticky=tk.W, pady=2)
        cred_frame = ttk.Frame(conn_group)
        cred_frame.grid(row=5, column=0, columnspan=2, sticky=tk.EW, pady=2)
        
        cred_entry = ttk.Entry(cred_frame, textvariable=self.cred_path)
        cred_entry.pack(side=tk.LEFT, fill=tk.X, expand=True)
        
        browse_btn = ttk.Button(cred_frame, text="Examinar", width=10, command=self.browse_credentials)
        browse_btn.pack(side=tk.RIGHT, padx=(5,0))
        
        # Estado de conexión
        status_group = ttk.LabelFrame(parent, text="Estado de Conexión", padding=10)
        status_group.pack(fill=tk.X, padx=10, pady=5)
        
        status_label = ttk.Label(status_group, textvariable=self.connection_status, style='Status.TLabel')
        status_label.pack(pady=5)
        
        # Botones de conexión
        btn_frame = ttk.Frame(status_group)
        btn_frame.pack(fill=tk.X, pady=5)
        
        connect_btn = ttk.Button(btn_frame, text="Conectar", command=self.connect_firebase)
        connect_btn.pack(side=tk.LEFT, padx=(0,5))
        
        disconnect_btn = ttk.Button(btn_frame, text="Desconectar", command=self.disconnect_firebase)
        disconnect_btn.pack(side=tk.LEFT)

    def create_manual_control(self, parent):
        """Control manual del robot"""
        # Título
        title_group = ttk.LabelFrame(parent, text="Control Manual del Robot", padding=10)
        title_group.pack(fill=tk.X, padx=10, pady=5)
        
        # Marco para botones de dirección
        direction_frame = ttk.Frame(title_group)
        direction_frame.pack(pady=10)
        
        # Velocidad manual
        speed_frame = ttk.Frame(title_group)
        speed_frame.pack(pady=5)
        ttk.Label(speed_frame, text="Velocidad:").pack(side=tk.LEFT)
        self.manual_speed = tk.IntVar(value=100)
        ttk.Scale(speed_frame, from_=50, to=250, variable=self.manual_speed, 
                orient=tk.HORIZONTAL, length=150).pack(side=tk.LEFT, padx=10)
        speed_label = ttk.Label(speed_frame, text="100")
        speed_label.pack(side=tk.LEFT)
        self.manual_speed.trace_add('write', lambda *args: speed_label.configure(text=str(self.manual_speed.get())))
        
        # Disposición de botones en cruz
        btn_up = ttk.Button(direction_frame, text="↑", width=3, command=lambda: self.manual_command(self.manual_speed.get(), 0, 0))
        btn_up.grid(row=0, column=1, padx=2, pady=2)
        
        btn_left = ttk.Button(direction_frame, text="←", width=3, command=lambda: self.manual_command(0, -self.manual_speed.get(), 0))
        btn_left.grid(row=1, column=0, padx=2, pady=2)
        
        btn_right = ttk.Button(direction_frame, text="→", width=3, command=lambda: self.manual_command(0, self.manual_speed.get(), 0))
        btn_right.grid(row=1, column=2, padx=2, pady=2)
        
        btn_down = ttk.Button(direction_frame,text="↓", width=3, command=lambda: self.manual_command(-self.manual_speed.get(), 0, 0))
        btn_down.grid(row=2, column=1, padx=2, pady=2)

        # Botones de rotación
        rotate_frame = ttk.Frame(title_group)
        rotate_frame.pack(pady=10)
        
        ttk.Button(rotate_frame, text="↻ Girar Izq", command=lambda: self.manual_command(0, 0, -self.manual_speed.get())).pack(side=tk.LEFT, padx=5)
        ttk.Button(rotate_frame, text="↺ Girar Der", command=lambda: self.manual_command(0, 0, self.manual_speed.get())).pack(side=tk.LEFT, padx=5)
        
        # Botón de parada
        ttk.Button(title_group, text="⏹ PARAR", command=lambda: self.manual_command(0, 0, 0)).pack(pady=10)
        
    def create_main_buttons(self, parent):
        """Botones principales de control"""
        button_frame = ttk.Frame(parent)
        button_frame.pack(fill=tk.X, padx=10, pady=10)
        
        # Botón de iniciar/parar
        self.start_btn = ttk.Button(button_frame, text="INICIAR SISTEMA", 
                                   command=self.toggle_system, style='Accent.TButton')
        self.start_btn.pack(fill=tk.X, pady=5)
        
        # Botones secundarios
        btn_row1 = ttk.Frame(button_frame)
        btn_row1.pack(fill=tk.X, pady=2)
        
        save_btn = ttk.Button(btn_row1, text="Guardar Config", command=self.save_config)
        save_btn.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0,2))
        
        load_btn = ttk.Button(btn_row1, text="Cargar Config", command=self.load_config)
        load_btn.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(2,0))
        
        btn_row2 = ttk.Frame(button_frame)
        btn_row2.pack(fill=tk.X, pady=2)
        
        stop_robot_btn = ttk.Button(btn_row2, text="Parar Robot", command=self.stop_robot)
        stop_robot_btn.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0,2))
        
        clear_target_btn = ttk.Button(btn_row2, text="Limpiar Objetivo", command=self.clear_target)
        clear_target_btn.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(2,0))
        
        # Modo de orientación
        orient_frame = ttk.Frame(button_frame)
        orient_frame.pack(fill=tk.X, pady=5)
        
        orient_check = ttk.Checkbutton(orient_frame, text="Orientar hacia movimiento", 
                                     variable=self.orient_mode)
        orient_check.pack()
        
    def create_video_panel(self, parent):
        """Panel de video y estado del sistema"""
        # Título del panel
        video_title = ttk.Label(parent, text="Vista de Camara y Estado del Sistema", style='Title.TLabel')
        video_title.pack(pady=(10, 5))
        
        # Frame para video con scroll si es necesario
        video_container = ttk.Frame(parent, relief=tk.SUNKEN, borderwidth=2)
        video_container.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Label para mostrar el video
        self.video_label = ttk.Label(video_container, 
                                   text="Vista de camara aparecera aqui\ncuando se inicie el sistema\n\nClick para establecer objetivo", 
                                   font=('Arial', 12), 
                                   anchor=tk.CENTER,
                                   foreground='gray',
                                   background='black')
        self.video_label.pack(expand=True, fill=tk.BOTH)
        
        # Bind del click del mouse
        self.video_label.bind("<Button-1>", self.on_video_click)
        
        # Panel de estado
        status_frame = ttk.LabelFrame(parent, text="Estado del Sistema", padding=10)
        status_frame.pack(fill=tk.X, padx=10, pady=(0,10))
        
        # Crear grid de estado
        status_items = [
            ("Frecuencia de Camara:", self.current_fps),
            ("Posición del Robot:", self.robot_position),
            ("Objetivo Actual:", self.target_position),
            ("Estado Firebase:", self.connection_status)
        ]
        
        for i, (label_text, var) in enumerate(status_items):
            ttk.Label(status_frame, text=label_text).grid(row=i//2, column=(i%2)*2, sticky=tk.W, padx=5, pady=2)
            status_label = ttk.Label(status_frame, textvariable=var, style='Status.TLabel')
            status_label.grid(row=i//2, column=(i%2)*2+1, sticky=tk.W, padx=5, pady=2)
    
    def on_video_click(self, event):
        """Manejar click en el video - OPTIMIZADO"""
        if self.CURRENT_H_INV is None or not self.running:
            return
        
        # Obtener coordenadas relativas al widget
        widget_x = event.x
        widget_y = event.y
        
        # Calcular la escala y offset del video en el widget
        widget_width = self.video_label.winfo_width()
        widget_height = self.video_label.winfo_height()
        
        if self.current_frame is not None:
            frame_height, frame_width = self.current_frame.shape[:2]
            
            # Calcular escala manteniendo aspecto
            scale_x = widget_width / frame_width
            scale_y = widget_height / frame_height
            scale = min(scale_x, scale_y)
            
            # Calcular offset para centrar
            scaled_width = frame_width * scale
            scaled_height = frame_height * scale
            offset_x = (widget_width - scaled_width) / 2
            offset_y = (widget_height - scaled_height) / 2
            
            # Convertir coordenadas del widget a coordenadas del frame original
            if (widget_x >= offset_x and widget_x <= offset_x + scaled_width and
                widget_y >= offset_y and widget_y <= offset_y + scaled_height):
                
                frame_x = (widget_x - offset_x) / scale
                frame_y = (widget_y - offset_y) / scale
                
                # Convertir píxeles a coordenadas de la malla
                u, v_top = self.pix_to_grid_uv_fast((frame_x, frame_y), self.CURRENT_H_INV)
                u = np.clip(u, 0.0, 1.0)
                v = 1.0 - np.clip(v_top, 0.0, 1.0)
                
                gx = u * self.grid_n.get()
                gy = v * self.grid_n.get()
                
                # Establecer objetivo
                current_heading = self._current_robot_heading if self._robot_detected else None
                self.set_target(gx, gy, current_heading)
                print(f"[CLICK] Nuevo objetivo en ({gx:.2f}, {gy:.2f})")
        
    # ==========================
    #  Métodos de configuración
    # ==========================
    def set_resolution(self, width, height):
        """Establece resolución predefinida"""
        self.camera_width.set(width)
        self.camera_height.set(height)
        
    def browse_credentials(self):
        """Examinar archivo de credenciales"""
        from tkinter import filedialog
        filename = filedialog.askopenfilename(
            title="Seleccionar archivo de credenciales",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        if filename:
            self.cred_path.set(filename)
            
    def save_config(self):
        """Guardar configuración actual"""
        config = {
            'camera': {
                'index': self.camera_index.get(),
                'width': self.camera_width.get(),
                'height': self.camera_height.get(),
                'fps': self.camera_fps.get(),
            },
            'robot': {
                'id': self.robot_id.get(),
                'corner_ids': [var.get() for var in self.corner_ids],
                'grid_n': self.grid_n.get()
            },
            'control': {
                'kp_lin': self.kp_lin.get(),
                'kp_w_face': self.kp_w_face.get(),
                'kp_w_hold': self.kp_w_hold.get(),
                'cmd_max': self.cmd_max.get(),
                'w_max': self.w_max.get(),
                'pos_tolerance': self.pos_tolerance.get()
            },
            'firebase': {
                'url': self.firebase_url.get(),
                'path': self.firebase_path.get(),
                'cred_path': self.cred_path.get()
            },
            'advanced': {
                'max_lost_frames': getattr(self, 'max_lost_frames', tk.IntVar(value=15)).get(),
                'prediction_enabled': getattr(self, 'prediction_enabled', tk.BooleanVar(value=True)).get(),
                'quality_threshold': getattr(self, 'quality_threshold', tk.DoubleVar(value=30.0)).get()
            },
            'pi_config' : {
            'ki_lin': self.ki_lin.get(),
            'ki_w': self.ki_w.get(),
            'integral_max_lin': self.integral_max_lin.get(),
            'integral_max_w': self.integral_max_w.get(),
            'camera_delay': self.camera_delay.get(),
            'prediction_horizon': self.prediction_horizon.get()
            }
        }
        
        try:
            with open(self.config_file, 'w') as f:
                json.dump(config, f, indent=4)
            messagebox.showinfo("Éxito", f"Configuración guardada en {self.config_file}")
        except Exception as e:
            messagebox.showerror("Error", f"No se pudo guardar la configuración:\n{e}")
            
    def load_config(self):
        """Cargar configuración desde archivo"""
        if not os.path.exists(self.config_file):
            return
            
        try:
            with open(self.config_file, 'r') as f:
                config = json.load(f)
                
            # Cargar configuración de cámara
            if 'camera' in config:
                cam = config['camera']
                self.camera_index.set(cam.get('index', 1))
                self.camera_width.set(cam.get('width', 1920))
                self.camera_height.set(cam.get('height', 1080))
                self.camera_fps.set(cam.get('fps', 30))
                
            # Cargar configuración de robot
            if 'robot' in config:
                robot = config['robot']
                self.robot_id.set(robot.get('id', 4))
                corner_ids = robot.get('corner_ids', [1, 2, 5, 3])
                for i, corner_id in enumerate(corner_ids[:4]):
                    self.corner_ids[i].set(corner_id)
                self.grid_n.set(robot.get('grid_n', 10))
                
            # Cargar configuración de control
            if 'control' in config:
                ctrl = config['control']
                self.kp_lin.set(ctrl.get('kp_lin', 80.0))
                self.kp_w_face.set(ctrl.get('kp_w_face', 120.0))
                self.kp_w_hold.set(ctrl.get('kp_w_hold', 120.0))
                self.cmd_max.set(ctrl.get('cmd_max', 250))
                self.w_max.set(ctrl.get('w_max', 250))
                self.pos_tolerance.set(ctrl.get('pos_tolerance', 0.20))
                
            # Cargar configuración de Firebase
            if 'firebase' in config:
                fb = config['firebase']
                self.firebase_url.set(fb.get('url', ''))
                self.firebase_path.set(fb.get('path', ''))
                self.cred_path.set(fb.get('cred_path', ''))
            
            # Cargar configuración avanzada
            if 'advanced' in config:
                adv = config['advanced']
                if hasattr(self, 'max_lost_frames'):
                    self.max_lost_frames.set(adv.get('max_lost_frames', 15))
                if hasattr(self, 'prediction_enabled'):
                    self.prediction_enabled.set(adv.get('prediction_enabled', True))
                if hasattr(self, 'quality_threshold'):
                    self.quality_threshold.set(adv.get('quality_threshold', 30.0))
            
            if 'pi_control' in config:
                pi = config['pi_control']
                self.ki_lin.set(pi.get('ki_lin', 15.0))
                self.ki_w.set(pi.get('ki_w', 20.0))
                self.integral_max_lin.set(pi.get('integral_max_lin', 50.0))
                self.integral_max_w.set(pi.get('integral_max_w', 30.0))
                self.camera_delay.set(pi.get('camera_delay', 0.15))
                self.prediction_horizon.set(pi.get('prediction_horizon', 3))
                
        except Exception as e:
            messagebox.showerror("Error", f"No se pudo cargar la configuración:\n{e}")
            
    # ==========================
    #  Métodos de Firebase OPTIMIZADOS
    # ==========================
    def connect_firebase(self):
        """Conectar a Firebase"""
        try:
            if not os.path.exists(self.cred_path.get()):
                messagebox.showerror("Error", "Archivo de credenciales no encontrado")
                return
                
            _fb_init(_cred.Certificate(self.cred_path.get()), 
                    {'databaseURL': self.firebase_url.get()})
            
            instr_path = f"{self.firebase_path.get()}/instrucciones"
            self.instr_ref = _db.reference(instr_path)
            self.firebase_connected = True
            
            # Iniciar hilo de Firebase
            self.firebase_thread = threading.Thread(target=self.firebase_worker, daemon=True)
            self.firebase_thread.start()
            
            self.connection_status.set("Conectado")
            messagebox.showinfo("Éxito", "Conectado a Firebase correctamente")
            
        except Exception as e:
            self.connection_status.set("Error de conexión")
            messagebox.showerror("Error Firebase", f"No se pudo conectar a Firebase:\n{e}")
    
    def firebase_worker(self):
        """Hilo separado para procesar comandos de Firebase - EVITA BLOQUEOS"""
        while self.running and self.firebase_connected:
            try:
                # Procesar comandos de la cola con timeout
                try:
                    cmd_data = self.firebase_queue.get(timeout=0.1)
                    if cmd_data is None:  # Señal de parada
                        break
                    
                    # Enviar comando a Firebase
                    if self.instr_ref:
                        if cmd_data['type'] == 'movement':
                            self.instr_ref.child("movimiento").update({
                                "vx": str(cmd_data['vx']), 
                                "vy": str(cmd_data['vy'])
                            })
                        elif cmd_data['type'] == 'rotation':
                            self.instr_ref.child("rotación").update({
                                "w": str(cmd_data['w'])
                            })
                        elif cmd_data['type'] == 'stop':
                            self.instr_ref.update({"parar": True})
                            self.instr_ref.child("movimiento").update({"vx": "0", "vy": "0"})
                            self.instr_ref.child("rotación").update({"w": "0"})
                    
                    self.firebase_queue.task_done()
                    
                except queue.Empty:
                    continue
                    
            except Exception as e:
                print(f"[FIREBASE][ERR] Error en worker: {e}")
                time.sleep(0.1)
    
    def disconnect_firebase(self):
        """Desconectar Firebase"""
        self.firebase_connected = False
        if self.firebase_thread:
            # Enviar señal de parada al hilo
            try:
                self.firebase_queue.put_nowait(None)
            except queue.Full:
                pass
        self.instr_ref = None
        self.connection_status.set("Desconectado")
        
    # ==========================
    #  SISTEMA DE SEGUIMIENTO ROBUSTO - NUEVO
    # ==========================
    def update_robot_tracker(self, detected, position=None, heading=None):
        """Actualizar el estado del seguidor del robot"""
        if detected:
            self._robot_tracker['detected'] = True
            self._robot_tracker['lost_frames'] = 0
            
            if position is not None:
                # Actualizar historial de posiciones
                self._robot_tracker['position_history'].append(position)
                self._robot_tracker['last_position'] = position
                
                # Calcular velocidad si tenemos suficiente historial
                if len(self._robot_tracker['position_history']) >= 2:
                    pos_prev = self._robot_tracker['position_history'][-2]
                    pos_curr = self._robot_tracker['position_history'][-1]
                    velocity = (pos_curr[0] - pos_prev[0], pos_curr[1] - pos_prev[1])
                    self._robot_tracker['velocity_history'].append(velocity)
            
            if heading is not None:
                self._robot_tracker['last_heading'] = heading
                
            # Actualizar estado en GUI
            if hasattr(self, 'tracking_status'):
                self.root.after(0, lambda: self.tracking_status.set("Detectado"))
                self.root.after(0, lambda: self.frames_lost.set("0"))
                
        else:
            self._robot_tracker['lost_frames'] += 1
            max_lost = getattr(self, 'max_lost_frames', tk.IntVar(value=15)).get()
            
            if self._robot_tracker['lost_frames'] >= max_lost:
                self._robot_tracker['detected'] = False
                
                # Actualizar estado en GUI
                if hasattr(self, 'tracking_status'):
                    self.root.after(0, lambda: self.tracking_status.set("Perdido"))
            else:
                # Actualizar contador de frames perdidos
                if hasattr(self, 'frames_lost'):
                    self.root.after(0, lambda: self.frames_lost.set(str(self._robot_tracker['lost_frames'])))
    
    def predict_robot_position(self):
        """Predecir posición del robot basada en el historial"""
        if not getattr(self, 'prediction_enabled', tk.BooleanVar(value=True)).get():
            return None
            
        if (len(self._robot_tracker['velocity_history']) >= 2 and 
            self._robot_tracker['last_position'] is not None):
            
            # Usar velocidad promedio de las últimas mediciones
            recent_velocities = list(self._robot_tracker['velocity_history'])[-3:]
            avg_vx = sum(v[0] for v in recent_velocities) / len(recent_velocities)
            avg_vy = sum(v[1] for v in recent_velocities) / len(recent_velocities)
            
            # Predecir posición basada en frames perdidos
            frames_lost = self._robot_tracker['lost_frames']
            if frames_lost > 0 and frames_lost < 10:  # Solo predecir hasta 10 frames
                last_pos = self._robot_tracker['last_position']
                predicted_x = last_pos[0] + avg_vx * frames_lost
                predicted_y = last_pos[1] + avg_vy * frames_lost
                
                # Limitar predicción a los límites de la malla
                predicted_x = np.clip(predicted_x, 0, self.grid_n.get())
                predicted_y = np.clip(predicted_y, 0, self.grid_n.get())
                
                return (predicted_x, predicted_y)
        
        return None
    
    def assess_frame_quality(self, frame):
        """Evaluar la calidad del frame para detección ArUco"""
        try:
            # Convertir a escala de grises
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Calcular la varianza del Laplaciano (medida de nitidez)
            laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()
            
            return laplacian_var
        except:
            return 0.0
    
    def enhance_frame_for_detection(self, frame):
        """Mejorar frame para mejor detección ArUco"""
        try:
            # Aplicar filtro bilateral para reducir ruido manteniendo bordes
            enhanced = cv2.bilateralFilter(frame, 9, 75, 75)
            
            # Mejorar contraste usando CLAHE
            gray = cv2.cvtColor(enhanced, cv2.COLOR_BGR2GRAY)
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
            enhanced_gray = clahe.apply(gray)
            
            # Convertir de vuelta a BGR
            enhanced = cv2.cvtColor(enhanced_gray, cv2.COLOR_GRAY2BGR)
            
            return enhanced
        except:
            return frame
    
    # ==========================
    #  Métodos del sistema principal MEJORADOS
    # ==========================
    def toggle_system(self):
        """Iniciar/parar el sistema"""
        if not self.running:
            self.start_system()
        else:
            self.stop_system()
            
    def start_system(self):
        """Iniciar el sistema de detección y control"""
        try:
            # Abrir cámara
            self.cap = self.open_camera()
            
            # Reiniciar seguidor
            self._robot_tracker = {
                'detected': False,
                'lost_frames': 0,
                'max_lost_frames': getattr(self, 'max_lost_frames', tk.IntVar(value=15)).get(),
                'last_position': None,
                'last_heading': None,
                'velocity_history': deque(maxlen=5),
                'position_history': deque(maxlen=10),
                'prediction_enabled': True
            }
            
            # Cambiar botón
            self.start_btn.configure(text="PARAR SISTEMA")
            
            # Iniciar hilo de cámara
            self.running = True
            self.camera_thread = threading.Thread(target=self.camera_loop_enhanced, daemon=True)
            self.camera_thread.start()
            
            messagebox.showinfo("Sistema Iniciado", "Sistema mejorado iniciado correctamente")
            
        except Exception as e:
            messagebox.showerror("Error", f"No se pudo iniciar el sistema:\n{e}")
            
    def stop_system(self):
        """Parar el sistema"""
        self.running = False
        
        if self.cap:
            self.cap.release()
            self.cap = None
            
        # Limpiar el video label
        self.video_label.configure(image='', text="Vista de camara aparecera aqui\ncuando se inicie el sistema\n\nClick para establecer objetivo")
        self.current_frame = None
        
        self.start_btn.configure(text="INICIAR SISTEMA")
        self.current_fps.set("0.0 Hz")
        self.robot_position.set("No detectado")
        
        # Actualizar estado de seguimiento
        if hasattr(self, 'tracking_status'):
            self.tracking_status.set("Inactivo")
            self.frames_lost.set("0")
        
        messagebox.showinfo("Sistema Detenido", "Sistema detenido correctamente")
        
    def open_camera(self):
        """Abrir cámara con configuración mejorada"""
        backends = [cv2.CAP_DSHOW, cv2.CAP_MSMF]
        
        cap = None
        for backend in backends:
            cap = cv2.VideoCapture(self.camera_index.get(), backend)
            if cap.isOpened():
                break
            cap.release()
                
        if not cap.isOpened():
            raise RuntimeError("No se pudo abrir la cámara")
            
        # Configurar cámara con parámetros optimizados para ArUco
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Buffer mínimo para baja latencia
        cap.set(cv2.CAP_PROP_FPS, self.camera_fps.get())
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width.get())
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height.get())
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # Exposición manual
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # Enfoque manual
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)  # Activa exposición automática
        cap.set(cv2.CAP_PROP_BRIGHTNESS, -1)       # Intenta establecer brillo automático (si es compatible)

        
        # Parámetros adicionales para mejor detección
        cap.set(cv2.CAP_PROP_CONTRAST, 0.5)
        cap.set(cv2.CAP_PROP_SATURATION, 0.5)
        cap.set(cv2.CAP_PROP_GAMMA, 100)
        
        return cap
        
    def camera_loop_enhanced(self):
        """Loop principal de la cámara MEJORADO con seguimiento robusto"""
        fps_history = deque(maxlen=20)
        fps_counter = 0
        current_fps = 0.0
        misses = 0
        MAX_MISSES = 10
        
        # Variables para procesamiento optimizado
        last_fps_update = 0
        frame_enhancement_counter = 0
        
        while self.running:
            loop_start = time.time()
            
            ret, frame = self.cap.read()
            if not ret or frame is None:
                misses += 1
                if misses > MAX_MISSES:
                    print("[WARN] Reintentando abrir cámara...")
                    self.cap.release()
                    time.sleep(0.1)
                    try:
                        self.cap = self.open_camera()
                    except:
                        self.root.after(0, lambda: messagebox.showerror("Error", "Se perdió la conexión con la cámara"))
                        break
                    misses = 0
                continue
            misses = 0
            
            # Evaluar calidad del frame
            frame_quality = self.assess_frame_quality(frame)
            quality_threshold = getattr(self, 'quality_threshold', tk.DoubleVar(value=30.0)).get()
            
            # Mejorar frame si la calidad es baja (pero no siempre para rendimiento)
            if frame_quality < quality_threshold:
                frame_enhancement_counter += 1
                if frame_enhancement_counter % 3 == 0:  # Solo cada 3 frames de baja calidad
                    frame = self.enhance_frame_for_detection(frame)
            else:
                frame_enhancement_counter = 0
            
            # Calcular FPS real
            fps_counter += 1
            fps_history.append(loop_start)
            
            if loop_start - last_fps_update > 0.5:
                if len(fps_history) > 1:
                    time_span = fps_history[-1] - fps_history[0]
                    if time_span > 0:
                        current_fps = (len(fps_history) - 1) / time_span
                        self.root.after(0, lambda fps=current_fps: self.current_fps.set(f"{fps:.1f} Hz"))
                last_fps_update = loop_start
            
            # Procesar detección robusta
            self.process_frame_robust(frame, loop_start, frame_quality)
            
            # Guardar frame actual para el click handler
            self.current_frame = frame.copy()
            
            # Mostrar frame en tkinter
            self.display_frame_tkinter(frame)
            
            # Control de velocidad del bucle más agresivo para mejor seguimiento
            time.sleep(0.001)  # Pausa mínima
        
    def process_frame_robust(self, frame, current_time, frame_quality):
        """Procesamiento robusto del frame con seguimiento predictivo"""
        # Detección ArUco con parámetros mejorados
        corners, ids, _ = self.detector.detectMarkers(frame)
        
        robot_detected = False
        robot_position = None
        robot_heading = None
        
        if ids is not None:
            ids = ids.flatten()
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            
            # Procesar esquinas de la mesa
            centers_px = {}
            corner_ids_list = [var.get() for var in self.corner_ids]
            
            for i, mid in enumerate(ids):
                if mid in corner_ids_list:
                    c = corners[i][0]
                    centers_px[int(mid)] = c.mean(axis=0)
            
            # Actualizar homografía de manera más estable
            H, H_inv = self.update_homography_stable(centers_px, current_time)
            self.CURRENT_H = H
            self.CURRENT_H_INV = H_inv
            
            if H is not None:
                self.draw_grid_consistent(frame, H)
            
            # Procesar robot con seguimiento mejorado
            robot_id = self.robot_id.get()
            if (robot_id in ids) and (H_inv is not None):
                robot_detected = True
                robot_position, robot_heading = self.process_robot_robust(
                    frame, corners, ids, robot_id, H_inv, current_time
                )
        
        # Actualizar seguidor del robot
        self.update_robot_tracker(robot_detected, robot_position, robot_heading)
        
        # Si perdimos el robot, usar predicción si está habilitada
        if not robot_detected and self._robot_tracker['lost_frames'] < self._robot_tracker['max_lost_frames']:
            predicted_pos = self.predict_robot_position()
            if predicted_pos is not None:
                # Dibujar robot predicho
                px = self.grid_to_pix_fast(predicted_pos[0], predicted_pos[1])
                if px is not None:
                    cv2.circle(frame, px, 8, (255, 128, 0), 2)  # Naranja para predicción
                    cv2.putText(frame, f"PRED({predicted_pos[0]:.1f},{predicted_pos[1]:.1f})",
                               (px[0]+15, px[1]-15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,128,0), 2)
                    
                    # Usar posición predicha para control si tenemos objetivo
                    if self._target_g is not None and (current_time - self._last_command_time > self._command_interval):
                        last_heading = self._robot_tracker.get('last_heading', 0.0)
                        self.goto_controller_pi_with_delay_compensation(
                            predicted_pos[0], predicted_pos[1], last_heading, current_time
                        )
                        self._last_command_time = current_time
                        
                        # Actualizar estado de predicción
                        if hasattr(self, 'prediction_status'):
                            self.root.after(0, lambda: self.prediction_status.set("Activa"))
        else:
            if hasattr(self, 'prediction_status'):
                self.root.after(0, lambda: self.prediction_status.set("Inactiva"))
        
        # Dibujar elementos UI
        if current_time - self._last_gui_update_time > self._gui_update_interval:
            self.draw_target_marker(frame)
            self.draw_hud_enhanced(frame, frame_quality)
            self._last_gui_update_time = current_time
        else:
            if self._target_g is not None:
                self.draw_target_marker_simple(frame)
                
        # Si perdimos completamente el robot, parar
        if not self._robot_tracker['detected'] and self._target_g is not None:
            self.stop_robot()
            print("[TRACKING] Robot perdido - Deteniendo movimiento")
    
    def process_robot_robust(self, frame, corners, ids, robot_id, H_inv, current_time):
        """Procesamiento robusto del robot detectado"""
        idx = list(ids).index(robot_id)
        c_robot = corners[idx][0]
        center_px = c_robot.mean(axis=0)
        px, py = int(center_px[0]), int(center_px[1])
        
        # Coordenadas en la malla
        u, v_top = self.pix_to_grid_uv_fast((px, py), H_inv)
        u = np.clip(u, 0.0, 1.0)
        v = 1.0 - np.clip(v_top, 0.0, 1.0)
        
        gx = u * self.grid_n.get()
        gy = v * self.grid_n.get()
        
        # Calcular ángulo del robot con mayor precisión
        p_dir_img = (c_robot[0] + 0.5*(c_robot[1] - c_robot[0]))
        u0, v0_top = self.pix_to_grid_uv_fast(center_px, H_inv)
        u1, v1_top = self.pix_to_grid_uv_fast(p_dir_img, H_inv)
        
        du, dv = (u1 - u0), (1.0 - v1_top) - (1.0 - v0_top)
        angle_grid = np.degrees(np.arctan2(dv, du))
        self._current_robot_heading = angle_grid
        
        # Visualizar robot con información mejorada
        cv2.circle(frame, (px, py), 10, (0,0,255), -1)
        cv2.circle(frame, (px, py), 15, (0,255,0), 2)  # Anillo verde para indicar detección activa
        
        # Dibujar dirección del robot
        arrow_end = (int(px + 25 * np.cos(np.radians(angle_grid))), 
                    int(py + 25 * np.sin(np.radians(angle_grid))))
        cv2.arrowedLine(frame, (px, py), arrow_end, (0,0,255), 3, tipLength=0.3)
        
        if current_time - self._last_gui_update_time > self._gui_update_interval:
            cv2.putText(frame, f"R({gx:.1f},{gy:.1f}) {self._current_robot_heading:.0f}°",
                       (px+20, py-20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
            cv2.putText(frame, "DETECTADO", (px+20, py+35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
        
        self._last_robot_px = (px, py)
        self._robot_detected = True
        
        # Actualizar estado en GUI
        if current_time - self._last_gui_update_time > self._gui_update_interval:
            pos_text = f"({gx:.2f}, {gy:.2f}) @ {self._current_robot_heading:.1f}°"
            self.root.after(0, lambda: self.robot_position.set(pos_text))
        
        # Control del robot con mejor manejo de comandos
        if self._target_g is not None and (current_time - self._last_command_time > self._command_interval):
            self.goto_controller_pi_with_delay_compensation(gx, gy, self._current_robot_heading, current_time)
            self._last_command_time = current_time
        
        return (gx, gy), angle_grid
    
    def update_homography_stable(self, centers_px, current_time):
        """Actualizar homografía de manera más estable"""
        # Recalcular con menos frecuencia pero más estabilidad
        if current_time - self._homography_cache["timestamp"] < 0.3:  # Cada 300ms
            if (self._homography_cache["H"] is not None and 
                self._homography_cache["stability_counter"] >= self._homography_cache["min_stability"]):
                return self._homography_cache["H"], self._homography_cache["H_inv"]
        
        if len(centers_px) == 4:
            try:
                pts_dst = self.order_quad([centers_px[k] for k in centers_px.keys()])
                pts_src = np.array([[0.,0.],[1.,0.],[1.,1.],[0.,1.]], dtype=np.float32)
                H = cv2.getPerspectiveTransform(pts_src, pts_dst)
                H_inv = np.linalg.inv(H)
                
                # Verificar estabilidad comparando con homografía anterior
                is_stable = True
                if self._homography_cache["H"] is not None:
                    diff = np.linalg.norm(H - self._homography_cache["H"])
                    is_stable = diff < 0.1  # Umbral de estabilidad
                
                if is_stable:
                    self._homography_cache["stability_counter"] += 1
                else:
                    self._homography_cache["stability_counter"] = 0
                
                self._homography_cache.update({
                    "H": H,
                    "H_inv": H_inv,
                    "timestamp": current_time
                })
                
                return H, H_inv
                
            except Exception as e:
                print(f"[WARN] Error calculando homografía: {e}")
        
        return self._homography_cache["H"], self._homography_cache["H_inv"]
    
    def draw_hud_enhanced(self, img, frame_quality):
        """HUD mejorado con información de calidad y seguimiento"""
        hud_lines = [
            f"Robot ID: {self.robot_id.get()}",
            f"Calidad: {frame_quality:.1f}",
            f"Seguimiento: {'ON' if self._robot_tracker['detected'] else 'OFF'}",
            "Click para objetivo"
        ]
        
        y_start = 25
        for i, line in enumerate(hud_lines):
            y = y_start + i * 20
            color = (0,255,0) if "ON" in line else (255,255,255)
            cv2.putText(img, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        # Indicador de predicción si está activa
        if (not self._robot_tracker['detected'] and 
            self._robot_tracker['lost_frames'] > 0 and 
            self._robot_tracker['lost_frames'] < 10):
            cv2.putText(img, f"PREDICIENDO ({self._robot_tracker['lost_frames']})", 
                       (10, y_start + 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,128,0), 2)
    
    def draw_grid_consistent(self, img, H):
        """Dibujar malla de forma consistente"""
        if H is None:
            return
            
        try:
            N = self.grid_n.get()
            step = 1.0 / N
            
            # Líneas principales cada 2 para rendimiento
            for i in range(0, N+1, 2):
                u = i * step
                src = np.array([[[u, 0.0]], [[u, 1.0]]], dtype=np.float32)
                dst = cv2.perspectiveTransform(src, H)
                p1, p2 = tuple(dst[0,0].astype(int)), tuple(dst[1,0].astype(int))
                cv2.line(img, p1, p2, (0,255,0), 1)
            
            for i in range(0, N+1, 2):
                v = i * step
                src = np.array([[[0.0, v]], [[1.0, v]]], dtype=np.float32)
                dst = cv2.perspectiveTransform(src, H)
                p1, p2 = tuple(dst[0,0].astype(int)), tuple(dst[1,0].astype(int))
                cv2.line(img, p1, p2, (0,255,0), 1)

            # Borde siempre visible
            box = np.array([[[0,0]], [[1,0]], [[1,1]], [[0,1]], [[0,0]]], dtype=np.float32)
            box_t = cv2.perspectiveTransform(box, H).astype(int)
            cv2.polylines(img, [box_t.reshape(-1,2)], isClosed=True, color=(0,180,255), thickness=2)
            
            # Ejes de referencia
            ax = np.array([[[0,0]], [[0.15,0]]], dtype=np.float32)
            ay = np.array([[[0,0]], [[0,0.15]]], dtype=np.float32)
            ax_t = cv2.perspectiveTransform(ax, H).astype(int)
            ay_t = cv2.perspectiveTransform(ay, H).astype(int)
            cv2.arrowedLine(img, tuple(ax_t[0,0]), tuple(ax_t[1,0]), (255,80,80), 2, tipLength=0.1)
            cv2.arrowedLine(img, tuple(ay_t[0,0]), tuple(ay_t[1,0]), (80,80,255), 2, tipLength=0.1)
            
        except Exception as e:
            print(f"[WARN] Error dibujando malla: {e}")
    
    def order_quad(self, pts):
        """Ordenar cuadrilátero"""
        pts = np.asarray(pts, dtype=np.float32)
        center = pts.mean(axis=0)
        angles = np.arctan2(pts[:, 1] - center[1], pts[:, 0] - center[0])
        sorted_indices = np.argsort(angles)
        return pts[sorted_indices]
    
    def draw_target_marker(self, img):
        """Dibujar marcador de objetivo mejorado"""
        if self._target_g is None or self.CURRENT_H is None:
            return

        px = self.grid_to_pix_fast(self._target_g[0], self._target_g[1])
        if px is None:
            return

        x, y = px
        # Marcador animado
        import time
        pulse = int(8 + 4 * math.sin(time.time() * 3))  # Pulso animado
        
        cv2.circle(img, (x, y), pulse, (0, 255, 255), 3)
        cv2.line(img, (x-20, y), (x+20, y), (0, 255, 255), 2)
        cv2.line(img, (x, y-20), (x, y+20), (0, 255, 255), 2)
        cv2.putText(img, f"OBJETIVO ({self._target_g[0]:.1f},{self._target_g[1]:.1f})",
                    (x+25, y-25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
        
        # Línea robot -> objetivo con información de distancia
        if self._last_robot_px is not None:
            cv2.line(img, self._last_robot_px, px, (0, 200, 255), 2)
            # Calcular y mostrar distancia
            if self._robot_tracker['last_position'] is not None:
                dist = math.hypot(
                    self._target_g[0] - self._robot_tracker['last_position'][0],
                    self._target_g[1] - self._robot_tracker['last_position'][1]
                )
                mid_x = (self._last_robot_px[0] + x) // 2
                mid_y = (self._last_robot_px[1] + y) // 2
                cv2.putText(img, f"d={dist:.2f}", (mid_x, mid_y), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,200,255), 1)
    
    def draw_target_marker_simple(self, img):
        """Dibujar marcador de objetivo simple"""
        if self._target_g is None or self.CURRENT_H is None:
            return

        px = self.grid_to_pix_fast(self._target_g[0], self._target_g[1])
        if px is None:
            return

        x, y = px
        cv2.circle(img, (x, y), 8, (0, 255, 255), 2)
    
    def display_frame_tkinter(self, frame):
        """Mostrar frame en tkinter - optimizado"""
        try:
            widget_width = self.video_label.winfo_width()
            widget_height = self.video_label.winfo_height()
            
            if widget_width <= 1:
                widget_width = 640
            if widget_height <= 1:
                widget_height = 480
                
            frame_height, frame_width = frame.shape[:2]
            
            scale_x = widget_width / frame_width
            scale_y = widget_height / frame_height
            scale = min(scale_x, scale_y)
            
            new_width = int(frame_width * scale)
            new_height = int(frame_height * scale)
            
            if new_width > 0 and new_height > 0:
                resized_frame = cv2.resize(frame, (new_width, new_height), interpolation=cv2.INTER_LINEAR)
                rgb_frame = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2RGB)
                pil_image = Image.fromarray(rgb_frame)
                photo = ImageTk.PhotoImage(pil_image)
                
                self.root.after(0, self.update_video_label, photo)
                
        except Exception as e:
            print(f"[ERROR] Error mostrando frame: {e}")
    
    def update_video_label(self, photo):
        """Actualizar el label del video en el hilo principal"""
        try:
            self.video_label.configure(image=photo, text="")
            self.video_label.image = photo
        except Exception as e:
            print(f"[ERROR] Error actualizando video label: {e}")
    
    def pix_to_grid_uv_fast(self, pt_pix, H_inv):
        """Conversión rápida pixel -> UV"""
        if H_inv is None:
            return 0.0, 0.0
        try:
            x, y = pt_pix[0], pt_pix[1]
            w = H_inv[2,0]*x + H_inv[2,1]*y + H_inv[2,2]
            if abs(w) < 1e-8:
                return 0.0, 0.0
            u = (H_inv[0,0]*x + H_inv[0,1]*y + H_inv[0,2]) / w
            v = (H_inv[1,0]*x + H_inv[1,1]*y + H_inv[1,2]) / w
            return float(u), float(v)
        except:
            return 0.0, 0.0
    
    def grid_to_pix_fast(self, gx, gy):
        """Conversión rápida grid -> pixel"""
        if self.CURRENT_H is None:
            return None
        try:
            u = float(gx) / float(self.grid_n.get())
            v_top = 1.0 - (float(gy) / float(self.grid_n.get()))
            
            H = self.CURRENT_H
            w = H[2,0]*u + H[2,1]*v_top + H[2,2]
            if abs(w) < 1e-8:
                return None
            x = (H[0,0]*u + H[0,1]*v_top + H[0,2]) / w
            y = (H[1,0]*u + H[1,1]*v_top + H[1,2]) / w
            return int(x), int(y)
        except:
            return None
    
    # ==========================
    #  Métodos de control del robot MEJORADOS
    # ==========================
    def set_target(self, gx_star, gy_star, current_heading_deg=None):
        """Establecer objetivo"""
        self._target_g = (float(gx_star), float(gy_star))
        
        if not self.orient_mode.get() and current_heading_deg is not None:
            self._hold_heading_rad = math.radians(current_heading_deg)
        
        target_text = f"({self._target_g[0]:.2f}, {self._target_g[1]:.2f})"
        self.root.after(0, lambda: self.target_position.set(target_text))
        
        print(f"[GOTO] Nuevo objetivo: {target_text}")
    
    def clear_target(self):
        """Limpiar objetivo"""
        self._target_g = None
        self._hold_heading_rad = None
        self.stop_robot()
        self.root.after(0, lambda: self.target_position.set("Sin objetivo"))
    
    def manual_command(self, vx, vy, w):
        """Enviar comando manual al robot"""
        if not self.firebase_connected:
            messagebox.showwarning("Sin Conexión", "Conecta a Firebase primero")
            return
        
        try:
            # Limpiar objetivo automático si existe
            self._target_g = None
            self._hold_heading_rad = None
            
            # Enviar comando directamente
            self.firebase_queue.put_nowait({
                'type': 'movement',
                'vx': int(vx),
                'vy': int(vy)
            })
            self.firebase_queue.put_nowait({
                'type': 'rotation', 
                'w': int(w)
            })
            
            print(f"[MANUAL] vx={vx} vy={vy} w={w}")
            
        except queue.Full:
            print("[MANUAL][WARN] Cola Firebase llena")
        except Exception as e:
            print(f"[MANUAL][ERROR] {e}")
    
    def stop_robot(self):
        """Parar robot"""
        if not self.firebase_connected:
            return
        try:
            self.firebase_queue.put_nowait({'type': 'stop'})
            print("[GOTO] Robot detenido")
        except queue.Full:
            print("[GOTO][WARN] Cola Firebase llena, comando descartado")
    
    def _send_cmd_optimized(self, vx, vy, w):
        """Enviar comando al robot - NO BLOQUEA"""
        if not self.firebase_connected:
            return
        try:
            self.firebase_queue.put_nowait({
                'type': 'movement',
                'vx': int(vx),
                'vy': int(vy)
            })
            self.firebase_queue.put_nowait({
                'type': 'rotation',
                'w': int(w)
            })
        except queue.Full:
            print("[GOTO][WARN] Cola Firebase llena, comando descartado")
    
    def _sat(self, x, lim):
        """Saturar valor"""
        return max(-lim, min(lim, x))
    
    def goto_controller_pi_with_delay_compensation(self, gx, gy, heading_deg, current_time):
        """
        Controlador PI con compensación de delay de cámara
        Reemplaza a goto_controller_step_optimized
        """
        if self._target_g is None:
            return

        # 1. PREDICCIÓN DE POSICIÓN FUTURA (compensar delay)
        predicted_pos = self.predict_future_position(gx, gy, heading_deg, current_time)
        if predicted_pos is not None:
            gx_pred, gy_pred, heading_pred = predicted_pos
        else:
            gx_pred, gy_pred, heading_pred = gx, gy, heading_deg

        # 2. CALCULAR ERRORES
        ex = self._target_g[0] - gx_pred
        ey = self._target_g[1] - gy_pred
        dist = math.hypot(ex, ey)

        # 3. VERIFICAR SI LLEGAMOS AL OBJETIVO
        if dist <= self.pos_tolerance.get():
            self._send_cmd_optimized(0, 0, 0)
            self.reset_integral_accumulators()  # Limpiar integradores
            if hasattr(self, '_target_reached_logged') and not self._target_reached_logged:
                print(f"[PI] Objetivo alcanzado: dist={dist:.3f}")
                self._target_reached_logged = True
            return
        
        self._target_reached_logged = False

        # 4. ACTUALIZAR INTEGRADORES (con dt adaptativo)
        dt = current_time - self._last_integral_time
        self._last_integral_time = current_time
        
        if dt > 0 and dt < 0.5:  # Validar dt razonable
            self._integral_ex += ex * dt
            self._integral_ey += ey * dt
            
            # Anti-windup: saturar integradores
            max_int = self.integral_max_lin.get()
            self._integral_ex = np.clip(self._integral_ex, -max_int, max_int)
            self._integral_ey = np.clip(self._integral_ey, -max_int, max_int)

        # 5. CONTROL PI LINEAL
        kp_factor = 1.0
        if self._robot_tracker['lost_frames'] > 0:
            kp_factor = max(0.3, 1.0 - (self._robot_tracker['lost_frames'] / 10.0))
        
        # Término proporcional
        vx_grid_p = self.kp_lin.get() * ex * kp_factor
        vy_grid_p = self.kp_lin.get() * ey * kp_factor
        
        # Término integral
        vx_grid_i = self.ki_lin.get() * self._integral_ex
        vy_grid_i = self.ki_lin.get() * self._integral_ey
        
        # Velocidad total en coordenadas de la malla
        vx_grid = vx_grid_p + vx_grid_i
        vy_grid = vy_grid_p + vy_grid_i

        # 6. TRANSFORMAR A COORDENADAS DEL ROBOT
        th = math.radians(heading_pred)
        
        vx_cmd = vx_grid * math.cos(th) + vy_grid * math.sin(th)
        vy_cmd = -vx_grid * math.sin(th) + vy_grid * math.cos(th)

        # 7. LIMITAR NORMA
        norm = math.hypot(vx_cmd, vy_cmd)
        cmd_max = self.cmd_max.get() * kp_factor
        if norm > cmd_max:
            scale = cmd_max / (norm + 1e-9)
            vx_cmd *= scale
            vy_cmd *= scale

        # 8. CONTROL PI ANGULAR
        if self.orient_mode.get():
            # Orientar hacia la dirección de movimiento
            if norm > 1e-3:
                th_des = math.atan2(vy_cmd, vx_cmd)
                e_th = math.atan2(math.sin(th_des - th), math.cos(th_des - th))
                
                # Actualizar integral angular
                if dt > 0 and dt < 0.5:
                    self._integral_w += e_th * dt
                    max_int_w = self.integral_max_w.get()
                    self._integral_w = np.clip(self._integral_w, -max_int_w, max_int_w)
                
                # PI angular
                w_cmd = (self.kp_w_face.get() * e_th + 
                        self.ki_w.get() * self._integral_w) * kp_factor
            else:
                w_cmd = 0.0
                self._integral_w *= 0.9  # Decaimiento suave
        else:
            # Mantener orientación fija
            if self._hold_heading_rad is None:
                self._hold_heading_rad = th
            e_th = math.atan2(math.sin(self._hold_heading_rad - th), 
                            math.cos(self._hold_heading_rad - th))
            
            # Actualizar integral angular
            if dt > 0 and dt < 0.5:
                self._integral_w += e_th * dt
                max_int_w = self.integral_max_w.get()
                self._integral_w = np.clip(self._integral_w, -max_int_w, max_int_w)
            
            # PI angular
            w_cmd = (self.kp_w_hold.get() * e_th + 
                    self.ki_w.get() * self._integral_w) * kp_factor

        w_cmd = self._sat(w_cmd, self.w_max.get() * kp_factor)

        # 9. REGISTRAR COMANDO EN HISTORIAL (para predicción futura)
        self._command_history.append({
            'time': current_time,
            'vx': vx_cmd,
            'vy': vy_cmd,
            'w': w_cmd
        })

        # 10. ENVIAR COMANDO
        self._send_cmd_optimized(round(-vx_cmd), round(vy_cmd), round(w_cmd))
        
        # 11. ACTUALIZAR ESTIMACIONES DE VELOCIDAD
        self.update_velocity_estimates(gx, gy, heading_deg, current_time)
        
        # 12. ACTUALIZAR GUI CON ESTADO DEL CONTROLADOR
        if current_time - self._last_gui_update_time > self._gui_update_interval:
            self.update_controller_status_display()

    # ==========================
    # PREDICCIÓN DE POSICIÓN FUTURA
    # ==========================

    def predict_future_position(self, gx, gy, heading_deg, current_time):
        """
        Predice la posición futura del robot compensando el delay de la cámara
        """
        delay = self.camera_delay.get()
        
        # Registrar posición actual
        self._position_history.append({
            'time': current_time,
            'gx': gx,
            'gy': gy,
            'heading': heading_deg
        })
        self._timestamp_history.append(current_time)
        
        # Si no tenemos suficiente historial, no predecir
        if len(self._position_history) < 3:
            return None
        
        # Estimar velocidad actual del robot
        if len(self._velocity_estimate) == 2 and np.linalg.norm(self._velocity_estimate) > 0:
            # Usar estimación de velocidad
            vx_est, vy_est = self._velocity_estimate
            w_est = self._angular_velocity_estimate
            
            # Predecir posición futura
            time_ahead = delay + (self.prediction_horizon.get() * 0.033)  # ~33ms por frame
            
            # Modelo cinemático simple
            heading_future = heading_deg + np.degrees(w_est * time_ahead)
            gx_future = gx + vx_est * time_ahead
            gy_future = gy + vy_est * time_ahead
            
            # Limitar a los bordes de la malla
            gx_future = np.clip(gx_future, 0, self.grid_n.get())
            gy_future = np.clip(gy_future, 0, self.grid_n.get())
            
            return (gx_future, gy_future, heading_future)
        
        return None

    def update_velocity_estimates(self, gx, gy, heading_deg, current_time):
        """
        Actualiza las estimaciones de velocidad basadas en el historial
        """
        if len(self._position_history) < 2:
            return
        
        # Obtener las últimas dos posiciones
        pos_curr = self._position_history[-1]
        pos_prev = self._position_history[-2]
        
        dt = pos_curr['time'] - pos_prev['time']
        
        if dt > 0 and dt < 0.5:  # Validar dt razonable
            # Velocidad lineal
            vx_est = (pos_curr['gx'] - pos_prev['gx']) / dt
            vy_est = (pos_curr['gy'] - pos_prev['gy']) / dt
            
            # Velocidad angular
            dheading = pos_curr['heading'] - pos_prev['heading']
            # Normalizar ángulo
            while dheading > 180:
                dheading -= 360
            while dheading < -180:
                dheading += 360
            w_est = np.radians(dheading) / dt
            
            # Filtro de suavizado (media móvil exponencial)
            alpha = 0.3
            self._velocity_estimate = alpha * np.array([vx_est, vy_est]) + \
                                    (1 - alpha) * self._velocity_estimate
            self._angular_velocity_estimate = alpha * w_est + \
                                            (1 - alpha) * self._angular_velocity_estimate

    # ==========================
    # FUNCIONES AUXILIARES
    # ==========================

    def reset_integral_accumulators(self):
        """Resetear acumuladores integrales (anti-windup manual)"""
        self._integral_ex = 0.0
        self._integral_ey = 0.0
        self._integral_w = 0.0
        print("[PI] Integradores reseteados")

    def update_controller_status_display(self):
        """Actualizar display del estado del controlador en la GUI"""
        # Calcular norma del error integral lineal
        i_lin_norm = math.hypot(self._integral_ex, self._integral_ey)
        
        self.integral_status.set(
            f"I_lin: {i_lin_norm:.2f}, I_ang: {self._integral_w:.2f}"
        )
        
        v_norm = np.linalg.norm(self._velocity_estimate)
        self.velocity_estimate_status.set(
            f"v_est: ({self._velocity_estimate[0]:.2f}, {self._velocity_estimate[1]:.2f}) = {v_norm:.2f}"
        )
    
    # ==========================
    #  Métodos de cierre
    # ==========================
    def on_closing(self):
        """Manejar cierre de aplicación"""
        if self.running:
            self.stop_system()
        
        # Detener Firebase
        self.disconnect_firebase()
        
        # Guardar configuración automáticamente
        try:
            self.save_config()
        except:
            pass
            
        self.root.destroy()

# ==========================
#  Función principal
# ==========================
def main():
    """Función principal de la aplicación"""
    # Configurar estilo moderno
    root = tk.Tk()
    
    # Configurar estilo ttk
    style = ttk.Style()
    if "vista" in style.theme_names():
        style.theme_use("vista")
    elif "clam" in style.theme_names():
        style.theme_use("clam")
    
    # Colores personalizados
    style.configure('Accent.TButton', 
                   font=('Arial', 10, 'bold'),
                   focuscolor='red')
    
    # Crear aplicación
    app = ArucoRobotGUI(root)
    
    # Centrar ventana
    root.update_idletasks()
    x = (root.winfo_screenwidth() // 2) - (root.winfo_width() // 2)
    y = (root.winfo_screenheight() // 2) - (root.winfo_height() // 2)
    root.geometry(f"+{x}+{y}")
    
    # Iniciar aplicación
    try:
        root.mainloop()
    except KeyboardInterrupt:
        print("\n[INFO] Aplicación cerrada por usuario")
    except Exception as e:
        print(f"[ERROR] Error inesperado: {e}")
        messagebox.showerror("Error Fatal", f"Error inesperado en la aplicación:\n{e}")

if __name__ == "__main__":
    main()