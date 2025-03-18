import asyncio
import websockets
import base64
import numpy as np
import cv2
import customtkinter as ctk
from PIL import Image, ImageTk
import threading
import tkintermapview
import torch

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

# Cargar el modelo YOLOv8
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
frame_width = 0
frame_height = 0
center_x, center_y = 0, 0
# Datos del hotwheels (medidas en mm)
CAR_LENGTH = 70  # Longitud
CAR_WIDTH = 70   # Ancho
CAR_HEIGHT = 20  # Altura
CAR_ASPECT_RATIO = CAR_LENGTH / CAR_WIDTH  # Relación de aspecto fija
# Datos de la cámara
SENSOR_WIDTH = 3.68  # mm
SENSOR_HEIGHT = 2.76  # mm
PIXEL_WIDTH = 1920 #Debe cuadrar con la resolución del frame de OpenCV
PIXEL_HEIGHT = 1080
FOCAL_LENGTH = 3.04  # mm

def estimate_distance(bbox_width, bbox_height):
    pixel_size_x = SENSOR_WIDTH / PIXEL_WIDTH  # Tamaño de un píxel en mm
    pixel_size_y = SENSOR_HEIGHT / PIXEL_HEIGHT  # Tamaño de un píxel en mm

    object_width_pixels = bbox_width
    object_height_pixels = bbox_height

    object_width_mm = object_width_pixels * pixel_size_x
    object_height_mm = object_height_pixels * pixel_size_y

    distance_x = (CAR_WIDTH * FOCAL_LENGTH) / object_width_mm
    distance_y = (CAR_HEIGHT * FOCAL_LENGTH) / object_height_mm

    return ((distance_x + distance_y) / 2)/10  # Promedio de ambas estimaciones en cm

# Función para ajustar el bounding box manteniendo la relación de aspecto
def adjust_bbox(x1, y1, x2, y2):
    bbox_width = x2 - x1
    bbox_height = y2 - y1

    if bbox_width / bbox_height > CAR_ASPECT_RATIO:
        # Ajustar la altura para mantener la relación de aspecto
        new_height = int(bbox_width / CAR_ASPECT_RATIO)
        y_center = (y1 + y2) // 2
        y1 = max(0, y_center - new_height // 2)
        y2 = min(frame_height, y_center + new_height // 2)
    else:
        # Ajustar el ancho para mantener la relación de aspecto
        new_width = int(bbox_height * CAR_ASPECT_RATIO)
        x_center = (x1 + x2) // 2
        x1 = max(0, x_center - new_width // 2)
        x2 = min(frame_width, x_center + new_width // 2)

    return x1, y1, x2, y2



class VideoClient(ctk.CTk):
    def __init__(self):
        super().__init__()

        self.title("WebSocket Video Stream")
        self.geometry("1280x800")

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

        self.map_widget = tkintermapview.TkinterMapView(self, corner_radius=2)
        self.map_widget.set_position(41.390205, 2.154007)
        self.map_widget.set_zoom(10)
        self.map_widget.grid(row=4, column=0, rowspan=2, columnspan=7, sticky="nsew")

        self.running = False
        self.video_loop = asyncio.new_event_loop()
        self.command_loop = asyncio.new_event_loop()

        threading.Thread(target=self.run_command_event_loop).start()

    def start_stream(self):
        self.running = True
        self.start_button.configure(state="disabled")
        self.stop_button.configure(state="normal")

        threading.Thread(target=self.run_video_event_loop).start()

    def stop_stream(self):
        self.running = False
        self.start_button.configure(state="normal")
        self.stop_button.configure(state="disabled")

    def run_video_event_loop(self):
        asyncio.set_event_loop(self.video_loop)
        self.video_loop.run_until_complete(self.receive_video())

    def run_command_event_loop(self):
        asyncio.set_event_loop(self.command_loop)
        self.command_loop.run_until_complete(self.connect_command_server())

    async def receive_video(self):
        video_uri = "ws://192.168.1.32:8765" # RasPi IP
        try:
            async with websockets.connect(video_uri) as video_websocket:
                self.video_websocket = video_websocket
                while self.running:
                    base64_frame = await video_websocket.recv()
                    frame_data = base64.b64decode(base64_frame)
                    frame_np = np.frombuffer(frame_data, np.uint8)
                    frame = cv2.imdecode(frame_np, cv2.IMREAD_COLOR)

                    if frame is not None:
                        self.update_image(frame)

        except Exception as e:
            print(f"Error receiving video: {e}")

    async def connect_command_server(self):
        command_uri = "ws://192.168.1.32:8766" # RasPi IP
        try:
            async with websockets.connect(command_uri) as command_websocket:
                self.command_websocket = command_websocket
                while True:
                    command = await command_websocket.recv()
                    print(f"Received command: {command}")

        except Exception as e:
            print(f"Error connecting to command server: {e}")

    def update_image(self, frame):
        global frame_width, frame_height, center_y, center_x

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        frame_width = 640
        frame_height = 480
        center_x, center_y = frame_width // 2, frame_height // 2

        # Detección de objetos con YOLO
        results = model(frame)
        detections = results.pandas().xyxy[0]  # Obtener las detecciones
        cars = detections[detections['name'] == 'car']

        if not cars.empty:
            # Seleccionar el coche más grande detectado
            largest_car = cars.loc[cars['confidence'].idxmax()]
            x1, y1, x2, y2 = int(largest_car['xmin']), int(largest_car['ymin']), int(
                largest_car['xmax']), int(
                largest_car['ymax'])

            # Ajustar bounding box para mantener la relación de aspecto fija
            x1, y1, x2, y2 = adjust_bbox(x1, y1, x2, y2)
            car_center_x, car_center_y = (x1 + x2) // 2, (y1 + y2) // 2

            # Calcular la distancia estimada
            bbox_width = x2 - x1
            bbox_height = y2 - y1
            distance = estimate_distance(bbox_width, bbox_height)
            print(f"Distancia estimada: {distance:.2f} cm")

            # Dibujar la caja delimitadora
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(frame, (car_center_x, car_center_y), 5, (0, 0, 255), -1)

            # Mover los servos para centrar el coche
            if car_center_x < center_x - 30: # Mover derecha
                self.send_command("left")
            elif car_center_x > center_x + 30:
                self.send_command("right")

            if car_center_y < center_y - 30: # Mover arriba
                self.send_command("up")
            elif car_center_y > center_y + 30:
                self.send_command("down")

        img = Image.fromarray(frame)
        #img = img.resize((800, 500))
        img_ctk = ctk.CTkImage(img, size=(640, 480))

        self.video_label.configure(image=img_ctk, text="")
        self.video_label.image = img_ctk

    def send_command(self, command):
        if self.command_websocket:
            asyncio.run_coroutine_threadsafe(self.command_websocket.send(command), self.command_loop)

if __name__ == "__main__":
    app = VideoClient()
    app.mainloop()
