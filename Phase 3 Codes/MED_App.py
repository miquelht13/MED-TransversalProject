import asyncio
import websockets
import base64
import numpy as np
import cv2
import customtkinter as ctk
from PIL import Image, ImageTk
import threading
import tkintermapview

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

class VideoClient(ctk.CTk):
    def __init__(self):
        super().__init__()

        self.title("WebSocket Video Stream")
        self.geometry("800x600")

        self.video_websocket = None
        self.command_websocket = None

        for i in range(6):
            self.grid_columnconfigure(i, weight=1)
            self.grid_rowconfigure(i, weight=1)

        self.video_label = ctk.CTkLabel(self, text="Waiting for video...")
        self.video_label.grid(row=0, column=0, rowspan=3, columnspan=3, sticky="nsew")

        self.start_button = ctk.CTkButton(self, text="Start Stream", command=self.start_stream)
        self.start_button.grid(row=0, column=3, sticky="nsew", padx=5, pady=5)

        self.stop_button = ctk.CTkButton(self, text="Stop Stream", command=self.stop_stream, state="disabled")
        self.stop_button.grid(row=0, column=4, sticky="nsew", padx=5, pady=5)

        self.up_button = ctk.CTkButton(self, text="Up", command=lambda: self.send_command("up"))
        self.up_button.grid(row=1, column=4, sticky="nsew", padx=5, pady=5)

        self.left_button = ctk.CTkButton(self, text="Left", command=lambda: self.send_command("left"))
        self.left_button.grid(row=2, column=3, sticky="nsew", padx=5, pady=5)

        self.track_button = ctk.CTkButton(self, text="Track/Untrack", command=lambda: self.send_command("track"))
        self.track_button.grid(row=2, column=4, sticky="nsew", padx=5, pady=5)

        self.right_button = ctk.CTkButton(self, text="Right", command=lambda: self.send_command("right"))
        self.right_button.grid(row=2, column=5, sticky="nsew", padx=5, pady=5)

        self.down_button = ctk.CTkButton(self, text="Down", command=lambda: self.send_command("down"))
        self.down_button.grid(row=3, column=4, sticky="nsew", padx=5, pady=5)

        self.map_widget = tkintermapview.TkinterMapView(self, corner_radius=2)
        self.map_widget.set_position(41.390205, 2.154007)
        self.map_widget.set_zoom(10)
        self.map_widget.grid(row=4, column=0, rowspan=2, columnspan=6, sticky="nsew")

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
        video_uri = "ws://localhost:8765"
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
        command_uri = "ws://localhost:8766"
        try:
            async with websockets.connect(command_uri) as command_websocket:
                self.command_websocket = command_websocket
                while True:
                    command = await command_websocket.recv()
                    print(f"Received command: {command}")

        except Exception as e:
            print(f"Error connecting to command server: {e}")

    def update_image(self, frame):
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(frame)
        img = img.resize((800, 500))
        img_ctk = ctk.CTkImage(img, size=(800, 500))

        self.video_label.configure(image=img_ctk, text="")
        self.video_label.image = img_ctk

    def send_command(self, command):
        if self.command_websocket:
            asyncio.run_coroutine_threadsafe(self.command_websocket.send(command), self.command_loop)

if __name__ == "__main__":
    app = VideoClient()
    app.mainloop()
