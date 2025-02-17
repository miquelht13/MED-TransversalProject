### Python test code for detecting a desired object. In this case, a car.
### Then, by giving the camera parameters and the known objecto to detect, computes the distance at which the object is from the camera

import cv2
import torch
import time

# Loads YOLOv5 model
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

# Camera config
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280) # Add camera resolution
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
frame_width = int(cap.get(3))
frame_height = int(cap.get(4))
center_x, center_y = frame_width // 2, frame_height // 2

# Measurements of the desired known object to detect
CAR_LENGTH = 14  # Length
CAR_WIDTH = 6   # Width
CAR_HEIGHT = 5.5  # Height
CAR_ASPECT_RATIO = CAR_LENGTH / CAR_WIDTH  # Aspect ratio

# Camera params
SENSOR_WIDTH = 3.68  # mm / cm / m
SENSOR_HEIGHT = 2.76  # mm / cm / m
PIXEL_WIDTH = 1280 # Must match with OpenCV config 
PIXEL_HEIGHT = 720
FOCAL_LENGTH = 3.04  # mm / cm / m

# Function to estimate the distance to the object
def estimate_distance(bbox_width, bbox_height):
    pixel_size_x = SENSOR_WIDTH / PIXEL_WIDTH  # Pixel size in mm 
    pixel_size_y = SENSOR_HEIGHT / PIXEL_HEIGHT  # Pixel size in mm 

    object_width_pixels = bbox_width
    object_height_pixels = bbox_height

    object_width_mm = object_width_pixels * pixel_size_x
    object_height_mm = object_height_pixels * pixel_size_y

    distance_x = (CAR_WIDTH * FOCAL_LENGTH) / object_width_mm
    distance_y = (CAR_HEIGHT * FOCAL_LENGTH) / object_height_mm

    return (distance_x + distance_y) / 2  # Average distance


# Function to adjust the bbox by keeping the apect ratio 
def adjust_bbox(x1, y1, x2, y2):
    bbox_width = x2 - x1
    bbox_height = y2 - y1

    if bbox_width / bbox_height > CAR_ASPECT_RATIO:
        # Adjust the height to keep the aspect ratio 
        new_height = int(bbox_width / CAR_ASPECT_RATIO)
        y_center = (y1 + y2) // 2
        y1 = max(0, y_center - new_height // 2)
        y2 = min(frame_height, y_center + new_height // 2)
    else:
        # Adjust the width (or length) to keep the aspect ratio 
        new_width = int(bbox_height * CAR_ASPECT_RATIO)
        x_center = (x1 + x2) // 2
        x1 = max(0, x_center - new_width // 2)
        x2 = min(frame_width, x_center + new_width // 2)

    return x1, y1, x2, y2


try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Object detection with YOLO
        results = model(frame)
        detections = results.pandas().xyxy[0]  # Save the detections 
        cars = detections[detections['name'] == 'car']

        if not cars.empty:
            # Keep the most trustable detection
            largest_car = cars.loc[cars['confidence'].idxmax()]
            x1, y1, x2, y2 = int(largest_car['xmin']), int(largest_car['ymin']), int(largest_car['xmax']), int(
                largest_car['ymax'])

            # Adjust the bbox 
            x1, y1, x2, y2 = adjust_bbox(x1, y1, x2, y2)
            car_center_x, car_center_y = (x1 + x2) // 2, (y1 + y2) // 2

            # Compute the ditance
            bbox_width = x2 - x1
            bbox_height = y2 - y1
            distance = estimate_distance(bbox_width, bbox_height)
            print(f"Distancia estimada: {distance:.2f} cm")

            # Draw the bbox
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(frame, (car_center_x, car_center_y), 5, (0, 0, 255), -1)

        # Show the images 
        cv2.imshow("Frame", frame)

        # Exit by pressing 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
except KeyboardInterrupt:
    print("Interrupción por teclado")
finally:
    cap.release()
    cv2.destroyAllWindows()
