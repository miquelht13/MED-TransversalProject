import cv2
from ultralytics import YOLO
import torch
import time
import threading

# Load the YOLO11 model
model = YOLO("drone_m.pt")

# Configuración de la cámara
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280) # Poner resolución raspi
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
frame_width = int(cap.get(3))
frame_height = int(cap.get(4))
center_x, center_y = frame_width // 2, frame_height // 2

# Datos del dron (Para este código sin importancia)
CAR_LENGTH = 14  # Longitud
CAR_WIDTH = 14   # Ancho
CAR_HEIGHT = 5.5  # Altura
CAR_ASPECT_RATIO = CAR_LENGTH / CAR_WIDTH  # Relación de aspecto fija

# Datos de la cámara
SENSOR_WIDTH = 3.68  # mm
SENSOR_HEIGHT = 2.76  # mm
PIXEL_WIDTH = 1280 #Debe cuadrar con la resolución del frame de OpenCV
PIXEL_HEIGHT = 720
FOCAL_LENGTH = 3.04  # mm

# Posiciones iniciales de los servos
servo_x_angle = 90
servo_y_angle = 90


# Función para mover el servo
def set_servo_angle(pwm, angle):
    duty = 2.5 + (angle / 18.0) * 10
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.05)

# Función para estimar la distancia al objeto
def estimate_distance(bbox_width, bbox_height):
    pixel_size_x = SENSOR_WIDTH / PIXEL_WIDTH  # Tamaño de un píxel en mm
    pixel_size_y = SENSOR_HEIGHT / PIXEL_HEIGHT  # Tamaño de un píxel en mm

    object_width_pixels = bbox_width
    object_height_pixels = bbox_height

    object_width_mm = object_width_pixels * pixel_size_x
    object_height_mm = object_height_pixels * pixel_size_y

    distance_x = (CAR_WIDTH * FOCAL_LENGTH) / object_width_mm
    distance_y = (CAR_HEIGHT * FOCAL_LENGTH) / object_height_mm

    return (distance_x + distance_y) / 2  # Promedio de ambas estimaciones en mm


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


def yolo_detection():
    global frame, detections, running
    while running:
        if frame is not None:
            results = model(frame)
            detections = results[0]


frame, detections, results, running = None, None,None, True
# Introducción de threads para tener el video fluido
yolo_thread = threading.Thread(target=yolo_detection, daemon=True)
yolo_thread.start()

try:

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Detección de objetos con YOLO 11
        #results = model(frame)

        #detections = results[0]  # Obtener las detecciones

        if detections and detections.boxes:
            # Seleccionar el coche más grande detectado
            # largest_car = detections.loc[detections['confidence'].idxmax()]
            for box in detections.boxes:
                x1, y1, x2, y2 = map(int,box.xyxy[0])

            # Ajustar bounding box para mantener la relación de aspecto fija
            x1, y1, x2, y2 = adjust_bbox(x1, y1, x2, y2)
            car_center_x, car_center_y = (x1 + x2) // 2, (y1 + y2) // 2

            # Calcular la distancia estimada
            bbox_width = x2 - x1
            bbox_height = y2 - y1
            distance = estimate_distance(bbox_width, bbox_height)
            print(f"Distancia estimada: {distance:.2f} cm")

            # Dibujar la caja delimitadora
            #frame = results[0].plot()
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(frame, (car_center_x, car_center_y), 5, (0, 0, 255), -1)

        # Mostrar la imagen con detecciones
        cv2.imshow("Drone Detection", frame)

        # Salir con la tecla 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            running = False
            break
except KeyboardInterrupt:
    print("Interrupción por teclado")
finally:
    cap.release()
    cv2.destroyAllWindows()
