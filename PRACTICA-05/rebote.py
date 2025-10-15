# -*- coding: utf-8 -*-
# ============================================
# CÓDIGO PARA PC CON CÁMARA ARUCO
# ============================================
# Este código corre en la PC y:
# 1. Detecta marcadores ArUco y delimita área
# 2. Rastrea posición del robot
# 3. Lee alertas del LIDAR desde Firebase
# 4. Controla movimiento autónomo evitando:
#    - Salir del área ArUco
#    - Obstáculos detectados por LIDAR
# 5. Envía comandos a Firebase
# ============================================

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
import random

# ==========================
#  Clase principal
# ==========================
class ArUcoControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Control ArUco - PC con Cámara")
        self.root.geometry("1200x750")
        self.root.configure(bg='#2b2b2b')
        
        # Variables del sistema
        self.cap = None
        self.running = False
        self.camera_thread = None
        self.firebase_thread = None
        self.config_file = "aruco_control_config.json"
        
        # Colas
        self.firebase_queue = queue.Queue(maxsize=10)
        
        # Visualización
        self.video_label = None
        self.current_frame = None
        
        # Configuración cámara
        self.camera_index = tk.IntVar(value=1)
        self.camera_width = tk.IntVar(value=1920)
        self.camera_height = tk.IntVar(value=1080)
        self.camera_fps = tk.IntVar(value=30)
        
        # Configuración marcadores
        self.robot_id = tk.IntVar(value=4)
        self.corner_ids = [tk.IntVar(value=1), tk.IntVar(value=2), 
                          tk.IntVar(value=5), tk.IntVar(value=3)]
        self.grid_n = tk.IntVar(value=10)
        
        # Control de velocidad
        self.speed = tk.IntVar(value=150)
        
        # Márgenes
        self.aruco_margin = tk.DoubleVar(value=1.0)
        
        # Estado
        self.current_fps = tk.StringVar(value="0.0 Hz")
        self.robot_position = tk.StringVar(value="No detectado")
        self.connection_status = tk.StringVar(value="Desconectado")
        self.movement_status = tk.StringVar(value="Detenido")
        self.lidar_status = tk.StringVar(value="LIDAR: Sin datos")
        self.obstacle_info = tk.StringVar(value="Sin obstáculos")
        
        # Firebase
        self.firebase_url = tk.StringVar(value="https://iot-app-f878d-default-rtdb.firebaseio.com/")
        self.firebase_path = tk.StringVar(value="robots/123456")
        self.cred_path = tk.StringVar(value="cred.json")
        self.instr_ref = None
        self.lidar_ref = None
        self.firebase_connected = False
        
        # Procesamiento
        self.CURRENT_H = None
        self.CURRENT_H_INV = None
        self._robot_detected = False
        self._last_command_time = 0
        self._command_interval = 0.05
        
        # ArUco detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.setup_aruco_params()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # Movimiento autónomo
        self.current_direction = random.uniform(0, 360)
        self.autonomous_enabled = False
        
        # Estado LIDAR (desde Firebase)
        self.lidar_obstacle = False
        self.lidar_distance = 3000
        self.lidar_free_direction = 'none'
        self.lidar_last_update = 0
        self.lidar_timeout = 2.0  # segundos sin datos LIDAR
        
        # Control de evasión
        self.in_evasion = False
        self.evasion_start = 0
        self.evasion_duration = 1.5
        self.evasion_direction = None
        
        # Cargar config
        self.load_config()
        
        # Crear interfaz
        self.create_interface()
        
        # Cierre
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
    
    def setup_aruco_params(self):
        """Configurar parámetros ArUco"""
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 23
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    
    def create_interface(self):
        """Crear interfaz"""
        # Panel izquierdo
        left_frame = ttk.Frame(self.root, relief=tk.RAISED, borderwidth=2)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, padx=10, pady=10)
        
        # Panel derecho
        right_frame = ttk.Frame(self.root, relief=tk.RAISED, borderwidth=2)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        self.create_control_panel(left_frame)
        self.create_video_panel(right_frame)
    
    def create_control_panel(self, parent):
        """Panel de controles"""
        title = ttk.Label(parent, text="Control ArUco + LIDAR", 
                         font=('Arial', 13, 'bold'))
        title.pack(pady=10)
        
        notebook = ttk.Notebook(parent)
        notebook.pack(fill=tk.BOTH, expand=True, padx=10)
        
        # Pestañas
        control_frame = ttk.Frame(notebook)
        notebook.add(control_frame, text="Control")
        self.create_control_tab(control_frame)
        
        config_frame = ttk.Frame(notebook)
        notebook.add(config_frame, text="Configuración")
        self.create_config_tab(config_frame)
        
        firebase_frame = ttk.Frame(notebook)
        notebook.add(firebase_frame, text="Firebase")
        self.create_firebase_tab(firebase_frame)
        
        self.create_main_buttons(parent)
    
    def create_control_tab(self, parent):
        """Pestaña de control"""
        # Velocidad
        speed_group = ttk.LabelFrame(parent, text="Velocidad", padding=15)
        speed_group.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Label(speed_group, text="Velocidad Robot:").grid(row=0, column=0, sticky=tk.W)
        speed_scale = ttk.Scale(speed_group, from_=50, to=250, variable=self.speed, orient=tk.HORIZONTAL, length=180)
        speed_scale.grid(row=1, column=0, columnspan=2, pady=5)
        
        self.speed_label = ttk.Label(speed_group, text="150", font=('Arial', 12, 'bold'))
        self.speed_label.grid(row=2, column=0, columnspan=2)
        self.speed.trace_add('write', lambda *args: self.speed_label.configure(text=str(self.speed.get())))
        
        # Margen ArUco
        margin_group = ttk.LabelFrame(parent, text="Margen de Seguridad ArUco", padding=15)
        margin_group.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Label(margin_group, text="Distancia al borde:").grid(row=0, column=0, sticky=tk.W)
        ttk.Spinbox(margin_group, from_=0.5, to=3.0, textvariable=self.aruco_margin, 
                   width=10, increment=0.1, format="%.1f").grid(row=0, column=1, padx=10)
        ttk.Label(margin_group, text="cuadros").grid(row=0, column=2)
        
        # Estado Robot
        robot_status = ttk.LabelFrame(parent, text="Estado Robot", padding=15)
        robot_status.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Label(robot_status, text="Posición:").grid(row=0, column=0, sticky=tk.W, pady=3)
        ttk.Label(robot_status, textvariable=self.robot_position, foreground='#4CAF50').grid(row=0, column=1, sticky=tk.W, pady=3)
        
        ttk.Label(robot_status, text="Movimiento:").grid(row=1, column=0, sticky=tk.W, pady=3)
        ttk.Label(robot_status, textvariable=self.movement_status, foreground='#2196F3').grid(row=1, column=1, sticky=tk.W, pady=3)
        
        # Estado LIDAR
        lidar_status = ttk.LabelFrame(parent, text="Estado LIDAR", padding=15)
        lidar_status.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Label(lidar_status, textvariable=self.lidar_status, font=('Arial', 9, 'bold')).pack(pady=3)
        ttk.Label(lidar_status, textvariable=self.obstacle_info, foreground='#FF5722').pack(pady=3)
    
    def create_config_tab(self, parent):
        """Pestaña configuración"""
        # Cámara
        cam_group = ttk.LabelFrame(parent, text="Cámara", padding=10)
        cam_group.pack(fill=tk.X, padx=10, pady=5)
        
        configs = [
            ("Índice:", self.camera_index, 0, 5),
            ("Ancho:", self.camera_width, 640, 1920),
            ("Alto:", self.camera_height, 480, 1080),
            ("FPS:", self.camera_fps, 15, 60)
        ]
        
        for i, (label, var, min_v, max_v) in enumerate(configs):
            ttk.Label(cam_group, text=label).grid(row=i, column=0, sticky=tk.W, pady=2)
            ttk.Spinbox(cam_group, from_=min_v, to=max_v, textvariable=var, width=10).grid(row=i, column=1, padx=10, pady=2)
        
        # Marcadores
        marker_group = ttk.LabelFrame(parent, text="Marcadores ArUco", padding=10)
        marker_group.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(marker_group, text="Robot ID:").grid(row=0, column=0, sticky=tk.W)
        ttk.Spinbox(marker_group, from_=0, to=50, textvariable=self.robot_id, width=10).grid(row=0, column=1, padx=10)
        
        ttk.Label(marker_group, text="Esquinas:").grid(row=1, column=0, sticky=tk.W, pady=2)
        corner_frame = ttk.Frame(marker_group)
        corner_frame.grid(row=1, column=1, pady=2)
        
        for i in range(4):
            ttk.Spinbox(corner_frame, from_=0, to=50, textvariable=self.corner_ids[i], width=5).pack(side=tk.LEFT, padx=2)
        
        ttk.Label(marker_group, text="Tamaño Malla:").grid(row=2, column=0, sticky=tk.W, pady=2)
        ttk.Spinbox(marker_group, from_=5, to=20, textvariable=self.grid_n, width=10).grid(row=2, column=1, padx=10, pady=2)
        
        # Botones
        btn_frame = ttk.Frame(parent)
        btn_frame.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Button(btn_frame, text="Guardar", command=self.save_config).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Cargar", command=self.load_config).pack(side=tk.LEFT, padx=5)
    
    def create_firebase_tab(self, parent):
        """Pestaña Firebase"""
        conn_group = ttk.LabelFrame(parent, text="Conexión", padding=10)
        conn_group.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(conn_group, text="URL:").grid(row=0, column=0, sticky=tk.W, pady=2)
        ttk.Entry(conn_group, textvariable=self.firebase_url, width=30).grid(row=1, column=0, columnspan=2, sticky=tk.EW, pady=2)
        
        ttk.Label(conn_group, text="Ruta:").grid(row=2, column=0, sticky=tk.W, pady=2)
        ttk.Entry(conn_group, textvariable=self.firebase_path, width=30).grid(row=3, column=0, columnspan=2, sticky=tk.EW, pady=2)
        
        ttk.Label(conn_group, text="Credenciales:").grid(row=4, column=0, sticky=tk.W, pady=2)
        cred_frame = ttk.Frame(conn_group)
        cred_frame.grid(row=5, column=0, columnspan=2, sticky=tk.EW, pady=2)
        
        ttk.Entry(cred_frame, textvariable=self.cred_path).pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(cred_frame, text="...", width=3, command=self.browse_credentials).pack(side=tk.RIGHT)
        
        # Estado
        status_group = ttk.LabelFrame(parent, text="Estado", padding=10)
        status_group.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(status_group, textvariable=self.connection_status, font=('Arial', 10, 'bold')).pack(pady=5)
        
        btn_frame = ttk.Frame(status_group)
        btn_frame.pack(pady=5)
        
        ttk.Button(btn_frame, text="Conectar", command=self.connect_firebase).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Desconectar", command=self.disconnect_firebase).pack(side=tk.LEFT, padx=5)
    
    def create_main_buttons(self, parent):
        """Botones principales"""
        button_frame = ttk.Frame(parent)
        button_frame.pack(fill=tk.X, padx=10, pady=10)
        
        self.start_btn = ttk.Button(button_frame, text="INICIAR SISTEMA", command=self.toggle_system)
        self.start_btn.pack(fill=tk.X, pady=5)
        
        self.autonomous_btn = ttk.Button(button_frame, text="ACTIVAR MOVIMIENTO", 
                                        command=self.toggle_autonomous, state=tk.DISABLED)
        self.autonomous_btn.pack(fill=tk.X, pady=5)
        
        ttk.Button(button_frame, text="PARAR ROBOT", command=self.stop_robot).pack(fill=tk.X, pady=5)
    
    def create_video_panel(self, parent):
        """Panel de video"""
        title = ttk.Label(parent, text="Vista de Cámara", font=('Arial', 12, 'bold'))
        title.pack(pady=5)
        
        video_container = ttk.Frame(parent, relief=tk.SUNKEN, borderwidth=2)
        video_container.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        self.video_label = ttk.Label(video_container, 
                                     text="Cámara se mostrará aquí", 
                                     font=('Arial', 12),
                                     background='black',
                                     foreground='gray')
        self.video_label.pack(expand=True, fill=tk.BOTH)
        
        # Estado
        status_frame = ttk.LabelFrame(parent, text="Estado del Sistema", padding=10)
        status_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(status_frame, text="FPS:").grid(row=0, column=0, sticky=tk.W, pady=2)
        ttk.Label(status_frame, textvariable=self.current_fps).grid(row=0, column=1, sticky=tk.W, pady=2)
    
    def browse_credentials(self):
        """Buscar credenciales"""
        from tkinter import filedialog
        filename = filedialog.askopenfilename(
            title="Seleccionar credenciales",
            filetypes=[("JSON", "*.json"), ("Todos", "*.*")]
        )
        if filename:
            self.cred_path.set(filename)
    
    # ==========================
    # CONFIG
    # ==========================
    def save_config(self):
        """Guardar configuración"""
        config = {
            'camera': {
                'index': self.camera_index.get(),
                'width': self.camera_width.get(),
                'height': self.camera_height.get(),
                'fps': self.camera_fps.get()
            },
            'markers': {
                'robot_id': self.robot_id.get(),
                'corner_ids': [v.get() for v in self.corner_ids],
                'grid_n': self.grid_n.get()
            },
            'control': {
                'speed': self.speed.get(),
                'aruco_margin': self.aruco_margin.get()
            },
            'firebase': {
                'url': self.firebase_url.get(),
                'path': self.firebase_path.get(),
                'cred_path': self.cred_path.get()
            }
        }
        
        try:
            with open(self.config_file, 'w') as f:
                json.dump(config, f, indent=4)
            messagebox.showinfo("Éxito", "Configuración guardada")
        except Exception as e:
            messagebox.showerror("Error", f"Error al guardar:\n{e}")
    
    def load_config(self):
        """Cargar configuración"""
        if not os.path.exists(self.config_file):
            return
        
        try:
            with open(self.config_file, 'r') as f:
                config = json.load(f)
            
            if 'camera' in config:
                self.camera_index.set(config['camera'].get('index', 1))
                self.camera_width.set(config['camera'].get('width', 1920))
                self.camera_height.set(config['camera'].get('height', 1080))
                self.camera_fps.set(config['camera'].get('fps', 30))
            
            if 'markers' in config:
                self.robot_id.set(config['markers'].get('robot_id', 4))
                corners = config['markers'].get('corner_ids', [1, 2, 5, 3])
                for i, cid in enumerate(corners[:4]):
                    self.corner_ids[i].set(cid)
                self.grid_n.set(config['markers'].get('grid_n', 10))
            
            if 'control' in config:
                self.speed.set(config['control'].get('speed', 150))
                self.aruco_margin.set(config['control'].get('aruco_margin', 1.0))
            
            if 'firebase' in config:
                self.firebase_url.set(config['firebase'].get('url', ''))
                self.firebase_path.set(config['firebase'].get('path', ''))
                self.cred_path.set(config['firebase'].get('cred_path', ''))
                
        except Exception as e:
            messagebox.showerror("Error", f"Error al cargar:\n{e}")
    
    # ==========================
    # FIREBASE
    # ==========================
    def connect_firebase(self):
        """Conectar Firebase"""
        try:
            if not os.path.exists(self.cred_path.get()):
                messagebox.showerror("Error", "Credenciales no encontradas")
                return
            
            _fb_init(_cred.Certificate(self.cred_path.get()), 
                    {'databaseURL': self.firebase_url.get()})
            
            base_path = self.firebase_path.get()
            self.instr_ref = _db.reference(f"{base_path}/instrucciones")
            self.lidar_ref = _db.reference(f"{base_path}/lidar")
            self.firebase_connected = True
            
            # Iniciar hilo Firebase
            self.firebase_thread = threading.Thread(target=self.firebase_worker, daemon=True)
            self.firebase_thread.start()
            
            # Iniciar hilo para leer LIDAR
            self.lidar_listener_thread = threading.Thread(target=self.lidar_listener, daemon=True)
            self.lidar_listener_thread.start()
            
            self.connection_status.set("✓ Conectado")
            messagebox.showinfo("Éxito", "Conectado a Firebase")
            
        except Exception as e:
            self.connection_status.set("✗ Error")
            messagebox.showerror("Error", f"No se pudo conectar:\n{e}")
    
    def disconnect_firebase(self):
        """Desconectar Firebase"""
        self.firebase_connected = False
        if self.firebase_thread:
            try:
                self.firebase_queue.put_nowait(None)
            except:
                pass
        self.instr_ref = None
        self.lidar_ref = None
        self.connection_status.set("Desconectado")
    
    def firebase_worker(self):
        """Worker Firebase"""
        while self.running and self.firebase_connected:
            try:
                cmd_data = self.firebase_queue.get(timeout=0.1)
                if cmd_data is None:
                    break
                
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
                print(f"[Firebase] Error: {e}")
    
    def lidar_listener(self):
        """Escucha datos del LIDAR desde Firebase"""
        while self.running and self.firebase_connected:
            try:
                if self.lidar_ref:
                    data = self.lidar_ref.get()
                    
                    if data:
                        self.lidar_obstacle = data.get('obstaculo_detectado', False)
                        self.lidar_distance = data.get('distancia_minima', 3000)
                        self.lidar_free_direction = data.get('direccion_libre', 'none')
                        self.lidar_last_update = time.time()
                        
                        # Actualizar GUI
                        if self.lidar_obstacle:
                            status = f"LIDAR: ⚠ Obstáculo {self.lidar_distance}mm"
                            info = f"Obstáculo detectado - Girar {self.lidar_free_direction}"
                        else:
                            status = "LIDAR: ✓ Camino libre"
                            info = "Sin obstáculos"
                        
                        self.root.after(0, lambda: self.lidar_status.set(status))
                        self.root.after(0, lambda: self.obstacle_info.set(info))
                
                time.sleep(0.1)
                
            except Exception as e:
                print(f"[LIDAR Listener] Error: {e}")
                time.sleep(1)
    
    # ==========================
    # SISTEMA
    # ==========================
    def toggle_system(self):
        """Toggle sistema"""
        if not self.running:
            self.start_system()
        else:
            self.stop_system()
    
    def start_system(self):
        """Iniciar sistema"""
        try:
            self.cap = self.open_camera()
            self.running = True
            
            self.start_btn.configure(text="PARAR SISTEMA")
            self.autonomous_btn.configure(state=tk.NORMAL)
            
            self.camera_thread = threading.Thread(target=self.camera_loop, daemon=True)
            self.camera_thread.start()
            
            messagebox.showinfo("Éxito", "Sistema iniciado")
        except Exception as e:
            messagebox.showerror("Error", f"No se pudo iniciar:\n{e}")
    
    def stop_system(self):
        """Parar sistema"""
        self.autonomous_enabled = False
        self.running = False
        
        if self.cap:
            self.cap.release()
            self.cap = None
        
        self.video_label.configure(image='', text="Cámara se mostrará aquí")
        self.current_frame = None
        
        self.start_btn.configure(text="INICIAR SISTEMA")
        self.autonomous_btn.configure(text="ACTIVAR MOVIMIENTO", state=tk.DISABLED)
        self.current_fps.set("0.0 Hz")
        self.robot_position.set("No detectado")
        self.movement_status.set("Detenido")
        
        self.stop_robot()
        messagebox.showinfo("Sistema", "Sistema detenido")
    
    def toggle_autonomous(self):
        """Toggle movimiento autónomo"""
        if not self.firebase_connected:
            messagebox.showwarning("Advertencia", "Conecta a Firebase primero")
            return
        
        self.autonomous_enabled = not self.autonomous_enabled
        
        if self.autonomous_enabled:
            self.autonomous_btn.configure(text="DESACTIVAR MOVIMIENTO")
            self.movement_status.set("Activo")
            self.current_direction = random.uniform(0, 360)
            self.in_evasion = False
            print(f"[AUTO] Activado - Dirección: {self.current_direction:.1f}°")
        else:
            self.autonomous_btn.configure(text="ACTIVAR MOVIMIENTO")
            self.movement_status.set("Detenido")
            self.stop_robot()
            self.in_evasion = False
            print("[AUTO] Desactivado")
    
    def open_camera(self):
        """Abrir cámara"""
        cap = cv2.VideoCapture(self.camera_index.get(), cv2.CAP_DSHOW)
        
        if not cap.isOpened():
            cap = cv2.VideoCapture(self.camera_index.get())
        
        if not cap.isOpened():
            raise RuntimeError("No se pudo abrir la cámara")
        
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width.get())
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height.get())
        cap.set(cv2.CAP_PROP_FPS, self.camera_fps.get())
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        return cap
    
    def camera_loop(self):
        """Loop de cámara"""
        fps_history = deque(maxlen=20)
        
        while self.running:
            loop_start = time.time()
            
            ret, frame = self.cap.read()
            if not ret or frame is None:
                continue
            
            # FPS
            fps_history.append(loop_start)
            if len(fps_history) > 1:
                time_span = fps_history[-1] - fps_history[0]
                if time_span > 0:
                    fps = (len(fps_history) - 1) / time_span
                    self.root.after(0, lambda f=fps: self.current_fps.set(f"{f:.1f} Hz"))
            
            # Procesar
            self.process_frame(frame, loop_start)
            
            # Guardar
            self.current_frame = frame.copy()
            
            # Mostrar
            self.display_frame(frame)
            
            time.sleep(0.001)
    
    def process_frame(self, frame, current_time):
        """Procesar frame"""
        # Detectar marcadores
        corners, ids, _ = self.detector.detectMarkers(frame)
        
        if ids is not None:
            ids = ids.flatten()
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            
            # Procesar esquinas
            centers_px = {}
            corner_ids_list = [v.get() for v in self.corner_ids]
            
            for i, mid in enumerate(ids):
                if mid in corner_ids_list:
                    c = corners[i][0]
                    centers_px[int(mid)] = c.mean(axis=0)
            
            # Homografía
            if len(centers_px) == 4:
                self.update_homography(centers_px)
            
            if self.CURRENT_H is not None:
                self.draw_grid(frame)
            
            # Procesar robot
            robot_id = self.robot_id.get()
            if robot_id in ids and self.CURRENT_H_INV is not None:
                self.process_robot(frame, corners, ids, robot_id, current_time)
        
        # HUD
        self.draw_hud(frame)
    
    def update_homography(self, centers_px):
        """Actualizar homografía"""
        try:
            pts_dst = self.order_quad([centers_px[k] for k in centers_px.keys()])
            pts_src = np.array([[0.,0.],[1.,0.],[1.,1.],[0.,1.]], dtype=np.float32)
            self.CURRENT_H = cv2.getPerspectiveTransform(pts_src, pts_dst)
            self.CURRENT_H_INV = np.linalg.inv(self.CURRENT_H)
        except:
            pass
    
    def order_quad(self, pts):
        """Ordenar cuadrilátero"""
        pts = np.asarray(pts, dtype=np.float32)
        center = pts.mean(axis=0)
        angles = np.arctan2(pts[:, 1] - center[1], pts[:, 0] - center[0])
        sorted_indices = np.argsort(angles)
        return pts[sorted_indices]
    
    def process_robot(self, frame, corners, ids, robot_id, current_time):
        """Procesar robot"""
        idx = list(ids).index(robot_id)
        c_robot = corners[idx][0]
        center_px = c_robot.mean(axis=0)
        px, py = int(center_px[0]), int(center_px[1])
        
        # Coordenadas malla
        u, v_top = self.pix_to_grid_uv((px, py))
        u = np.clip(u, 0.0, 1.0)
        v = 1.0 - np.clip(v_top, 0.0, 1.0)
        
        gx = u * self.grid_n.get()
        gy = v * self.grid_n.get()
        
        # Ángulo
        p_dir_img = (c_robot[0] + 0.5*(c_robot[1] - c_robot[0]))
        u0, v0_top = self.pix_to_grid_uv(center_px)
        u1, v1_top = self.pix_to_grid_uv(p_dir_img)
        
        du, dv = (u1 - u0), (1.0 - v1_top) - (1.0 - v0_top)
        angle_grid = np.degrees(np.arctan2(dv, du))
        
        # Visualizar
        cv2.circle(frame, (px, py), 10, (0, 255, 0), -1)
        cv2.circle(frame, (px, py), 15, (0, 0, 255), 2)
        
        arrow_end = (int(px + 30 * np.cos(np.radians(angle_grid))),
                    int(py + 30 * np.sin(np.radians(angle_grid))))
        cv2.arrowedLine(frame, (px, py), arrow_end, (0, 0, 255), 3, tipLength=0.3)
        
        cv2.putText(frame, f"R({gx:.1f},{gy:.1f}) {angle_grid:.0f}°",
                   (px+20, py-20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        self._robot_detected = True
        
        # Actualizar GUI
        pos_text = f"({gx:.2f}, {gy:.2f}) @ {angle_grid:.1f}°"
        self.root.after(0, lambda: self.robot_position.set(pos_text))
        
        # Control autónomo
        if self.autonomous_enabled and (current_time - self._last_command_time > self._command_interval):
            self.autonomous_control(gx, gy, angle_grid, current_time)
            self._last_command_time = current_time
    
    def autonomous_control(self, gx, gy, heading_deg, current_time):
        """Control autónomo integrado ArUco + LIDAR"""
        margin = self.aruco_margin.get()
        grid_max = self.grid_n.get()
        
        # Verificar si hay datos recientes del LIDAR
        lidar_active = (time.time() - self.lidar_last_update) < self.lidar_timeout
        
        # PRIORIDAD 1: Obstáculo LIDAR
        if lidar_active and self.lidar_obstacle and not self.in_evasion:
            print(f"[LIDAR] Obstáculo {self.lidar_distance}mm - Evasión {self.lidar_free_direction}")
            self.in_evasion = True
            self.evasion_start = current_time
            self.evasion_direction = self.lidar_free_direction
            
            # Cambiar dirección según LIDAR
            if self.lidar_free_direction == 'left':
                self.current_direction = (heading_deg + 90) % 360
            else:
                self.current_direction = (heading_deg - 90) % 360
        
        # PRIORIDAD 2: Límites ArUco
        dist_left = gx
        dist_right = grid_max - gx
        dist_bottom = gy
        dist_top = grid_max - gy
        
        near_aruco_edge = False
        
        if dist_left < margin:
            self.current_direction = random.uniform(-60, 60)
            near_aruco_edge = True
            print(f"[ArUco] Borde izquierdo - Nueva dirección: {self.current_direction:.1f}°")
        elif dist_right < margin:
            self.current_direction = random.uniform(120, 240)
            near_aruco_edge = True
            print(f"[ArUco] Borde derecho - Nueva dirección: {self.current_direction:.1f}°")
        elif dist_bottom < margin:
            self.current_direction = random.uniform(-30, 30)
            near_aruco_edge = True
            print(f"[ArUco] Borde inferior - Nueva dirección: {self.current_direction:.1f}°")
        elif dist_top < margin:
            self.current_direction = random.uniform(150, 210)
            near_aruco_edge = True
            print(f"[ArUco] Borde superior - Nueva dirección: {self.current_direction:.1f}°")
        
        # Esquinas ArUco
        if dist_left < margin and dist_bottom < margin:
            self.current_direction = random.uniform(0, 90)
            near_aruco_edge = True
        elif dist_right < margin and dist_bottom < margin:
            self.current_direction = random.uniform(90, 180)
            near_aruco_edge = True
        elif dist_left < margin and dist_top < margin:
            self.current_direction = random.uniform(270, 360)
            near_aruco_edge = True
        elif dist_right < margin and dist_top < margin:
            self.current_direction = random.uniform(180, 270)
            near_aruco_edge = True
        
        # Terminar evasión LIDAR
        if self.in_evasion and (current_time - self.evasion_start > 1.5):
            self.in_evasion = False
            print("[LIDAR] Evasión completada")
        
        # Calcular velocidades
        speed = self.speed.get()
        vx_grid = speed * np.cos(np.radians(self.current_direction))
        vy_grid = speed * np.sin(np.radians(self.current_direction))
        
        # Transformar a robot
        th = math.radians(heading_deg)
        vx_robot = vx_grid * math.cos(th) + vy_grid * math.sin(th)
        vy_robot = -vx_grid * math.sin(th) + vy_grid * math.cos(th)
        
        # Control angular
        angle_diff = self.current_direction - heading_deg
        while angle_diff > 180:
            angle_diff -= 360
        while angle_diff < -180:
            angle_diff += 360
        
        w = angle_diff * 3.0
        w = np.clip(w, -200, 200)
        
        # Enviar
        self.send_command(int(-vx_robot), int(vy_robot), int(w))
    
    def pix_to_grid_uv(self, pt_pix):
        """Convertir píxel a UV"""
        if self.CURRENT_H_INV is None:
            return 0.0, 0.0
        try:
            x, y = pt_pix[0], pt_pix[1]
            w = self.CURRENT_H_INV[2,0]*x + self.CURRENT_H_INV[2,1]*y + self.CURRENT_H_INV[2,2]
            if abs(w) < 1e-8:
                return 0.0, 0.0
            u = (self.CURRENT_H_INV[0,0]*x + self.CURRENT_H_INV[0,1]*y + self.CURRENT_H_INV[0,2]) / w
            v = (self.CURRENT_H_INV[1,0]*x + self.CURRENT_H_INV[1,1]*y + self.CURRENT_H_INV[1,2]) / w
            return float(u), float(v)
        except:
            return 0.0, 0.0
    
    def send_command(self, vx, vy, w):
        """Enviar comando"""
        if not self.firebase_connected:
            return
        try:
            self.firebase_queue.put_nowait({'type': 'movement', 'vx': vx, 'vy': vy})
            self.firebase_queue.put_nowait({'type': 'rotation', 'w': w})
        except queue.Full:
            pass
    
    def stop_robot(self):
        """Parar robot"""
        if not self.firebase_connected:
            return
        try:
            self.firebase_queue.put_nowait({'type': 'stop'})
        except queue.Full:
            pass
    
    def draw_grid(self, img):
        """Dibujar malla"""
        if self.CURRENT_H is None:
            return
        
        try:
            N = self.grid_n.get()
            step = 1.0 / N
            
            for i in range(0, N+1, 2):
                u = i * step
                src = np.array([[[u, 0.0]], [[u, 1.0]]], dtype=np.float32)
                dst = cv2.perspectiveTransform(src, self.CURRENT_H)
                p1 = tuple(dst[0,0].astype(int))
                p2 = tuple(dst[1,0].astype(int))
                cv2.line(img, p1, p2, (0, 255, 0), 1)
            
            for i in range(0, N+1, 2):
                v = i * step
                src = np.array([[[0.0, v]], [[1.0, v]]], dtype=np.float32)
                dst = cv2.perspectiveTransform(src, self.CURRENT_H)
                p1 = tuple(dst[0,0].astype(int))
                p2 = tuple(dst[1,0].astype(int))
                cv2.line(img, p1, p2, (0, 255, 0), 1)
            
            # Borde
            box = np.array([[[0,0]], [[1,0]], [[1,1]], [[0,1]], [[0,0]]], dtype=np.float32)
            box_t = cv2.perspectiveTransform(box, self.CURRENT_H).astype(int)
            cv2.polylines(img, [box_t.reshape(-1,2)], isClosed=True, color=(0, 255, 255), thickness=3)
            
            # Zona de seguridad ArUco
            margin = self.aruco_margin.get() / self.grid_n.get()
            safety_box = np.array([
                [[margin, margin]], 
                [[1-margin, margin]], 
                [[1-margin, 1-margin]], 
                [[margin, 1-margin]], 
                [[margin, margin]]
            ], dtype=np.float32)
            safety_t = cv2.perspectiveTransform(safety_box, self.CURRENT_H).astype(int)
            cv2.polylines(img, [safety_t.reshape(-1,2)], isClosed=True, 
                         color=(0, 165, 255), thickness=2)
            
        except Exception as e:
            print(f"[ERROR] Dibujar malla: {e}")
    
    def draw_hud(self, img):
        """Dibujar HUD"""
        lines = [
            f"Robot ID: {self.robot_id.get()}",
            f"Velocidad: {self.speed.get()}",
            f"Estado: {'ACTIVO' if self.autonomous_enabled else 'INACTIVO'}",
            f"Direccion: {self.current_direction:.1f}°" if self.autonomous_enabled else ""
        ]
        
        # Agregar info LIDAR si está activo
        if (time.time() - self.lidar_last_update) < self.lidar_timeout:
            if self.lidar_obstacle:
                lines.append(f"LIDAR: OBSTACULO {self.lidar_distance}mm")
            else:
                lines.append("LIDAR: OK")
        else:
            lines.append("LIDAR: SIN DATOS")
        
        # Estado evasión
        if self.in_evasion:
            lines.append("MODO: EVASION")
        
        y = 25
        for line in lines:
            if line:
                if 'OBSTACULO' in line or 'EVASION' in line:
                    color = (0, 0, 255)  # Rojo
                elif 'ACTIVO' in line or 'OK' in line:
                    color = (0, 255, 0)  # Verde
                else:
                    color = (255, 255, 255)  # Blanco
                
                cv2.putText(img, line, (10, y), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                y += 25
    
    def display_frame(self, frame):
        """Mostrar frame"""
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
                resized = cv2.resize(frame, (new_width, new_height))
                rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
                pil_img = Image.fromarray(rgb)
                photo = ImageTk.PhotoImage(pil_img)
                
                self.root.after(0, self.update_video_label, photo)
        except Exception as e:
            print(f"[ERROR] Mostrar frame: {e}")
    
    def update_video_label(self, photo):
        """Actualizar label"""
        try:
            self.video_label.configure(image=photo, text="")
            self.video_label.image = photo
        except:
            pass
    
    def on_closing(self):
        """Cerrar aplicación"""
        if self.running:
            self.stop_system()
        
        self.disconnect_firebase()
        
        try:
            self.save_config()
        except:
            pass
        
        self.root.destroy()

# ==========================
# FUNCIÓN PRINCIPAL
# ==========================
def main():
    """Iniciar aplicación"""
    root = tk.Tk()
    
    # Estilo
    style = ttk.Style()
    if "vista" in style.theme_names():
        style.theme_use("vista")
    elif "clam" in style.theme_names():
        style.theme_use("clam")
    
    app = ArUcoControlGUI(root)
    
    # Centrar ventana
    root.update_idletasks()
    x = (root.winfo_screenwidth() // 2) - (root.winfo_width() // 2)
    y = (root.winfo_screenheight() // 2) - (root.winfo_height() // 2)
    root.geometry(f"+{x}+{y}")
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        print("\n[INFO] Aplicación cerrada")
    except Exception as e:
        print(f"[ERROR] Error: {e}")

if __name__ == "__main__":
    main()