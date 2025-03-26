import asyncio
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

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

# Load the YOLO11 model
model = YOLO("drone_m.pt")
frame_width = 0
frame_height = 0
center_x, center_y = 0, 0
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

lat0, lon0 = 41.276254, 1.988224 #DroneLab facilities
bearing0, yaw, pitch = 40, 1500, 1850 
altitude0 = 1

# Earth's radius in meters
R = 6371000


def calculate_new_position(lat0d, lon0d, bearing0d, yawp, pitchp, distance, altitude0):
    # Convert angles to radians
    lat0 = math.radians(lat0d)
    lon0 = math.radians(lon0d)
    bearing0 = math.radians(bearing0d)
    yawd = 0 * 180/2000
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

        self.track_button = ctk.CTkButton(self, text="Track/Untrack", command=lambda: self.send_command("track"))
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

        self.running = False
        self.frame, self.detections, self.results, self.running_ia = None, None, None, True
        self.object_marker = None

        self.video_loop = asyncio.new_event_loop()
        self.command_loop = asyncio.new_event_loop()

        threading.Thread(target=self.run_command_event_loop).start()

        # Introducción de threads para tener el video fluido
        threading.Thread(target=self.yolo_detection).start()

    def yolo_detection(self):
        while self.running_ia:
            if self.frame is not None:
                self.results = model(self.frame)
                self.detections = self.results[0]

    def start_stream(self):
        self.running = True
        self.start_button.configure(state="disabled")
        self.stop_button.configure(state="normal")
        self.running_ia = True
        threading.Thread(target=self.run_video_event_loop).start()

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

        # Update the distance and height labels
        self.distance_label.configure(text=f"Drone at {distance:.2f} meters from sensor")
        self.height_label.configure(text=f"Drone at {height:.2f} meters height")

    def update_image(self, frame):
        global frame_width, frame_height, center_y, center_x, lat0, lon0, bearing0, yaw, pitch, altitude0

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        frame_width = 640
        frame_height = 480
        center_x, center_y = frame_width // 2, frame_height // 2

        # Detección de objetos con YOLO 11
        # results = model(frame)
        # detections = results[0]  # Obtener las detecciones

        if self.detections and self.detections.boxes:
            # Seleccionar el coche más grande detectado
            # largest_car = detections.loc[detections['confidence'].idxmax()]
            for box in self.detections.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])

            # Ajustar bounding box para mantener la relación de aspecto fija
            x1, y1, x2, y2 = adjust_bbox(x1, y1, x2, y2)
            drone_center_x, drone_center_y = (x1 + x2) // 2, (y1 + y2) // 2

            # Calcular la distancia estimada
            bbox_width = x2 - x1
            bbox_height = y2 - y1
            distance = estimate_distance(bbox_width, bbox_height)
            print(f"Distancia estimada: {distance:.2f} mm")

            # Sacar coordenadas del dron
            lat2, lon2, alt2, height = calculate_new_position(lat0, lon0, bearing0, yaw, pitch, distance, altitude0)
            # Update the marker position on the map and display distance and height
            self.update_marker(lat2, lon2, distance, height)

            # Dibujar la caja delimitadora
            # f = results[0].plot()
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(frame, (drone_center_x, drone_center_y), 5, (0, 0, 255), -1)

            # Mover los servos para centrar el coche
            if drone_center_x < center_x - 30: # Mover derecha
                self.send_command("left")
            elif drone_center_x > center_x + 30:
                self.send_command("right")

            if drone_center_y < center_y - 30: # Mover arriba
                self.send_command("up")
            elif drone_center_y > center_y + 30:
                self.send_command("down")

        img = Image.fromarray(frame)
        #img = img.resize((800, 500))
        img_ctk = ctk.CTkImage(img, size=(640, 480))

        self.video_label.configure(image=img_ctk, text="")
        self.video_label.image = img_ctk

    def send_command(self, command):
        print("Sending command:", command)
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
