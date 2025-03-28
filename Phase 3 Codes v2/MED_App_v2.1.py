import asyncio
import time

import websockets
import base64
import numpy as np
import cv2
import customtkinter as ctk
from PIL import Image, ImageTk
import threading
import tkintermapview
from ultralytics import YOLO
import math
import torch

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

# Configuración de YOLO con optimización
device = 'cuda' if torch.cuda.is_available() else 'cpu'
model = YOLO("drone_s.pt").to(device)
if torch.cuda.is_available():
    model = torch.compile(model)  # Optimizar el modelo si se usa GPU

frame_width = 0
frame_height = 0
center_x, center_y = 0, 0
frame_skip = 5  # Procesar cada 5 cuadros para mejorar la velocidad
frame_count = 0

# Datos del drone (medidas en mm)
DRONE_LENGTH = 250  # Longitud
DRONE_WIDTH = 250   # Ancho
DRONE_HEIGHT = 250  # Altura
DRONE_ASPECT_RATIO = DRONE_LENGTH / DRONE_WIDTH  # Relación de aspecto fija
# Datos de la cámara
SENSOR_WIDTH = 3.68  # mm
SENSOR_HEIGHT = 2.76  # mm
PIXEL_WIDTH = 1920 #Debe cuadrar con la resolución del frame de OpenCV
PIXEL_HEIGHT = 1080
FOCAL_LENGTH = 3.04  # mm

lat0, lon0 = 41.276254, 1.988224
bearing0, yaw, pitch = 40, 1500, 1850
altitude0 = 1

# Earth's radius in meters
R = 6371000


def calculate_new_position(lat0d, lon0d, bearing0d, yawp, pitchp, distance, altitude0):
    # Convert angles to radians
    lat0 = math.radians(lat0d)
    lon0 = math.radians(lon0d)
    bearing0 = math.radians(bearing0d)
    yawd = 0 * 180/2000 # Añadir estado neutro de los servos y calcular la regla de 3 de steps-grados del servo
    pitchd = 0 * 180 / 2000
    yaw = math.radians(yawd)
    pitch = math.radians(pitchd)
    distance = distance/1000

    # Calculate the new bearing angle (correct for wrapping)
    new_bearing = bearing0 + yaw
    new_bearing = (new_bearing + 2 * math.pi) % (2 * math.pi)  # Ensure within 0 to 2π

    # Calculate horizontal distance and altitude change
    d_horizontal = distance * math.cos(pitch)  # Ground distance
    delta_h = distance * math.sin(pitch)       # Altitude difference

    # Calculate new latitude
    lat2 = math.asin(math.sin(lat0) * math.cos(d_horizontal / R) +
                     math.cos(lat0) * math.sin(d_horizontal / R) * math.cos(new_bearing))

    # Calculate new longitude
    lon2 = lon0 + math.atan2(math.sin(new_bearing) * math.sin(d_horizontal / R) * math.cos(lat0),
                             math.cos(d_horizontal / R) - math.sin(lat0) * math.sin(lat2))

    # Convert latitude and longitude back to degrees
    lat2 = math.degrees(lat2)
    lon2 = math.degrees(lon2)

    # Calculate new altitude
    new_altitude = altitude0 + delta_h

    return lat2, lon2, new_altitude, delta_h



def estimate_distance(bbox_width, bbox_height):
    pixel_size_x = SENSOR_WIDTH / PIXEL_WIDTH  # Tamaño de un píxel en mm
    pixel_size_y = SENSOR_HEIGHT / PIXEL_HEIGHT  # Tamaño de un píxel en mm

    object_width_pixels = bbox_width
    object_height_pixels = bbox_height

    object_width_mm = object_width_pixels * pixel_size_x
    object_height_mm = object_height_pixels * pixel_size_y

    distance_x = (DRONE_WIDTH * FOCAL_LENGTH) / object_width_mm
    distance_y = (DRONE_HEIGHT * FOCAL_LENGTH) / object_height_mm

    return (distance_x + distance_y) / 2 # Promedio de ambas estimaciones en mm

# Función para ajustar el bounding box manteniendo la relación de aspecto
def adjust_bbox(x1, y1, x2, y2):
    bbox_width = x2 - x1
    bbox_height = y2 - y1

    if bbox_width / bbox_height > DRONE_ASPECT_RATIO:
        # Ajustar la altura para mantener la relación de aspecto
        new_height = int(bbox_width / DRONE_ASPECT_RATIO)
        y_center = (y1 + y2) // 2
        y1 = max(0, y_center - new_height // 2)
        y2 = min(frame_height, y_center + new_height // 2)
    else:
        # Ajustar el ancho para mantener la relación de aspecto
        new_width = int(bbox_height * DRONE_ASPECT_RATIO)
        x_center = (x1 + x2) // 2
        x1 = max(0, x_center - new_width // 2)
        x2 = min(frame_width, x_center + new_width // 2)

    return x1, y1, x2, y2


class VideoClient(ctk.CTk):
    def __init__(self):
        super().__init__()

        self.title("WebSocket Video Stream")
        self.geometry("1280x800")

        global lat0, lon0, bearing0, yaw, pitch, altitude0, R

        self.video_websocket = None
        self.command_websocket = None

        for i in range(7):
            self.grid_columnconfigure(i, weight=1)
            self.grid_rowconfigure(i, weight=1)

        self.video_label = ctk.CTkLabel(self, text="Waiting for video...")
        self.video_label.grid(row=0, column=0, rowspan=4, columnspan=3, sticky="nsew")

        self.start_button = ctk.CTkButton(self, text="Start Stream", command=self.start_stream)
        self.start_button.grid(row=0, column=4, sticky="nsew", padx=5, pady=5)

        self.stop_button = ctk.CTkButton(self, text="Stop Stream", command=self.stop_stream, state="disabled")
        self.stop_button.grid(row=0, column=5, sticky="nsew", padx=5, pady=5)

        self.neutral_button = ctk.CTkButton(self, text="Gimbal Neutral", command=lambda: self.send_command("neutral"))
        self.neutral_button.grid(row=0, column=6, sticky="nsew", padx=5, pady=5)

        self.up_button = ctk.CTkButton(self, text="Up", command=lambda: self.send_command("up"))
        self.up_button.grid(row=1, column=5, sticky="nsew", padx=5, pady=5)

        self.left_button = ctk.CTkButton(self, text="Left", command=lambda: self.send_command("left"))
        self.left_button.grid(row=2, column=4, sticky="nsew", padx=5, pady=5)

        self.track_button = ctk.CTkButton(self, text="Track", command=self.toggle_tracking)
        self.track_button.grid(row=2, column=5, sticky="nsew", padx=5, pady=5)

        self.right_button = ctk.CTkButton(self, text="Right", command=lambda: self.send_command("right"))
        self.right_button.grid(row=2, column=6, sticky="nsew", padx=5, pady=5)

        self.down_button = ctk.CTkButton(self, text="Down", command=lambda: self.send_command("down"))
        self.down_button.grid(row=3, column=5, sticky="nsew", padx=5, pady=5)

        self.distance_var = "Waiting object to detect..."
        self.distance_label = ctk.CTkLabel(self, text=self.distance_var)
        self.distance_label.grid(row=4, column=6, padx=5, pady=5, sticky="nsew")

        self.height_var = "Waiting object to detect..."
        self.height_label = ctk.CTkLabel(self, text=self.height_var)
        self.height_label.grid(row=5, column=6, padx=5, pady=5, sticky="nsew")

        self.map_widget = tkintermapview.TkinterMapView(self, corner_radius=2)
        self.map_widget.set_position(41.390205, 2.154007)
        self.map_widget.set_zoom(10)
        self.map_widget.grid(row=4, column=0, rowspan=2, columnspan=6, sticky="nsew")
        self.map_widget.add_right_click_menu_command(label="Add Gimbal Position",
                                                     command=self.set_gimbal_position,
                                                     pass_coords=True)

        self.running, self.tracking = False, False
        self.frame, self.detections, self.results, self.running_ia = None, None, None, True
        self.object_marker, self.gimbal_marker = None, None
        self.drone_center_x, self.drone_center_y = 0, 0

        # Inicializar el filtro de Kalman para suavizar el tracking [x, y, vx, vy]
        self.kalman = cv2.KalmanFilter(4, 2)
        self.kalman.measurementMatrix = np.array([[1, 0, 0, 0],
                                                  [0, 1, 0, 0]], np.float32)
        self.kalman.transitionMatrix = np.array([[1, 0, 1, 0],
                                                 [0, 1, 0, 1],
                                                 [0, 0, 1, 0],
                                                 [0, 0, 0, 1]], np.float32)
        self.kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03
        self.kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * 0.5
        self.kalman.errorCovPost = np.eye(4, dtype=np.float32)
        self.kalman_initialized = False

        self.video_loop = asyncio.new_event_loop()
        self.command_loop = asyncio.new_event_loop()

        threading.Thread(target=self.run_command_event_loop).start()

    def toggle_tracking(self):
        if not self.tracking:
            # Start tracking
            self.start_tracking()
            self.track_button.configure(text="Untrack")
        else:
            # Stop tracking
            self.stop_tracking()
            self.track_button.configure(text="Track")

    def start_tracking(self):
        self.tracking = True

    def stop_tracking(self):
        self.tracking = False

    def start_stream(self):
        global center_x, center_y
        self.running = True
        self.start_button.configure(state="disabled")
        self.stop_button.configure(state="normal")
        self.running_ia = True
        threading.Thread(target=self.run_video_event_loop).start()
        self.drone_center_x, self.drone_center_y = center_x, center_y

    def stop_stream(self):
        self.running = False
        self.running_ia = False
        self.start_button.configure(state="normal")
        self.stop_button.configure(state="disabled")

    def run_video_event_loop(self):
        asyncio.set_event_loop(self.video_loop)
        self.video_loop.run_until_complete(self.receive_video())

    def run_command_event_loop(self):
        asyncio.set_event_loop(self.command_loop)
        self.command_loop.run_until_complete(self.connect_command_server())

    async def receive_video(self):
        video_uri = "ws://127.0.0.1:8765" # RasPi IP ws://192.168.1.32:8765
        try:
            async with websockets.connect(video_uri) as video_websocket:
                self.video_websocket = video_websocket
                while self.running:
                    base64_frame = await video_websocket.recv()
                    frame_data = base64.b64decode(base64_frame)
                    frame_np = np.frombuffer(frame_data, np.uint8)
                    self.frame = cv2.imdecode(frame_np, cv2.IMREAD_COLOR)

                    if self.frame is not None:
                        self.update_image(self.frame)

        except Exception as e:
            print(f"Error receiving video: {e}")

    async def connect_command_server(self):
        command_uri = "ws://127.0.0.1:8766" # RasPi IP ws://192.168.1.32:8766
        try:
            async with websockets.connect(command_uri) as command_websocket:
                self.command_websocket = command_websocket
                while True:
                    command = await command_websocket.recv()
                    print(f"Received command: {command}")

        except Exception as e:
            print(f"Error connecting to command server: {e}")

    # Function to update the object marker's location on the map
    def update_marker(self, lat, lon, distance, height):
        distance = distance/1000
        if self.object_marker:
            # If marker already exists, update its position
            self.object_marker.set_position(lat, lon)
        else:
            # Create a new marker
            self.object_marker = self.map_widget.set_marker(lat, lon, text="Drone", marker_color_circle="red",
                                                            marker_color_outside="white")
            self.map_widget.set_position(lat, lon)
            self.map_widget.set_zoom(18)

        # Update the distance and height labels
        self.distance_label.configure(text=f"Drone at {distance:.2f} meters from sensor")
        self.height_label.configure(text=f"Drone at {height:.2f} meters height")

    def update_image(self, frame):
        global frame_width, frame_height, center_y, center_x, lat0, lon0, bearing0, yaw, pitch, altitude0, frame_count
        
        # Convertir el frame a RGB y establecer dimensiones fijas
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_width = 1920
        frame_height = 1080
        center_x, center_y = frame_width // 2, frame_height // 2
        
        if frame_count % frame_skip == 0:
            frame_tensor = torch.from_numpy(frame).to(device)  # Convertir a tensor y mover a GPU si está disponible
            results = model(frame_tensor)  # Ejecutar detección en GPU si es posible
            
            if results and results[0].boxes:
                best_box = None
                best_area = 0
                # Seleccionar el objeto de mayor área con confianza > 0.8
                for box in results[0].boxes:
                    conf = float(box.conf)
                    if conf < 0.8:
                        continue
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    area = (x2 - x1) * (y2 - y1)
                    if area > best_area:
                        best_area = area
                        best_box = (x1, y1, x2, y2)
                if best_box:
                    x1, y1, x2, y2 = adjust_bbox(*best_box)
                    # Calcular el centro detectado actual
                    detected_center_x = (x1 + x2) // 2
                    detected_center_y = (y1 + y2) // 2
            
                    # Preparar la medición para el filtro de Kalman
                    measurement = np.array([[np.float32(detected_center_x)],
                                            [np.float32(detected_center_y)]])
                    # Inicializar el estado del filtro si es la primera medición
                    if not self.kalman_initialized:
                        self.kalman.statePre = np.array([[np.float32(detected_center_x)],
                                                         [np.float32(detected_center_y)],
                                                         [0],
                                                         [0]], np.float32)
                        self.kalman_initialized = True
            
                    # Predicción y corrección del filtro de Kalman
                    prediction = self.kalman.predict()
                    corrected = self.kalman.correct(measurement)
                    self.drone_center_x, self.drone_center_y = int(corrected[0]), int(corrected[1])
            
                    bbox_width = x2 - x1
                    bbox_height = y2 - y1
                    distance = estimate_distance(bbox_width, bbox_height)
                    print(f"Distancia estimada: {distance:.2f} mm")
            
                    lat2, lon2, alt2, height = calculate_new_position(lat0, lon0, bearing0, yaw, pitch, distance, altitude0)
                    self.update_marker(lat2, lon2, distance, height)
            
                    # Dibujar la caja delimitadora y el centro filtrado del dron
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(frame, (self.drone_center_x, self.drone_center_y), 5, (0, 0, 255), -1)
            
                    # Calcular el error respecto al centro de la imagen para enviar comandos
                    error_x = center_x - self.drone_center_x
                    error_y = center_y - self.drone_center_y
                    threshold = 10  # Umbral en píxeles para emitir un comando
            
                    if self.tracking:
                        if error_x > threshold:
                            self.send_command("right")
                        elif error_x < -threshold:
                            self.send_command("left")
                        if error_y > threshold:
                            self.send_command("down")
                        elif error_y < -threshold:
                            self.send_command("up")

        frame_count += 1
        img = Image.fromarray(frame)
        img_ctk = ctk.CTkImage(img, size=(640, 480))
        self.video_label.configure(image=img_ctk, text="")
        self.video_label.image = img_ctk

    def send_command(self, command):
        if self.command_websocket:
            asyncio.run_coroutine_threadsafe(self.command_websocket.send(command), self.command_loop)

    def set_gimbal_position(self, coords):
        global lat0, lon0
        print("Setting gimbal position to:", coords)

        if hasattr(self, 'gimbal_marker') and self.gimbal_marker:
            self.gimbal_marker.delete()

        self.gimbal_marker = self.map_widget.set_marker(coords[0], coords[1], text="Gimbal", marker_color_circle="blue",
                                                        marker_color_outside="white")

        lat0, lon0 = coords[0], coords[1]

if __name__ == "__main__":
    app = VideoClient()
    app.mainloop()
