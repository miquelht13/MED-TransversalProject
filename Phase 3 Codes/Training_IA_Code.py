# Import required libraries
import numpy as np
import pandas as pd
import os
import matplotlib.pyplot as plt
from PIL import Image

# Install necessary packages (intended for Google Colab)
!pip install split-folders pillow ultralytics
!pip install roboflow

# Connect to Roboflow to download the dataset
from roboflow import Roboflow
rf = Roboflow(api_key="GpBrlvI4oqLgNVTUcJyj")  # Your Roboflow API key
project = rf.workspace("drone-srwzi").project("droneuni_dataset")  # Select the project
version = project.version(1)  # Choose version of the dataset
dataset = version.download("yolov11")  # Download dataset in YOLOv11 format

# Check if the dataset configuration file was downloaded successfully
file_path = '/content/DroneUni_Dataset-1/data.yaml'
if os.path.exists(file_path):
  print(f"The file at {file_path} exists.")
else:
  print(f"The file at {file_path} does not exist.")

# Display the current working directory and list its contents
cwd = os.getcwd()
print(f"Current working directory: {cwd}")
files = os.listdir(cwd)
print(f"Files and directories in '{cwd}': {files}")

# Remove unnecessary folder (usually comes with Colab)
dir_path = cwd + "/sample_data"
for files in os.listdir(dir_path):
  os.remove(dir_path + "/" + files)
os.rmdir(dir_path)

# Load the YOLOv11 model (nano version)
from ultralytics import YOLO
model = YOLO("yolo11n.pt")  # Load pre-trained YOLOv11 nano model
model.info()  # Display model architecture and details

# Set the dataset directory for training
from ultralytics import settings
settings.update({'datasets_dir': '/content/DroneUni_Dataset-1'})

# Train the model on the downloaded dataset
model.train(
    data='/content/DroneUni_Dataset-1/data.yaml',  # Path to data.yaml file
    epochs=100,  # Number of training epochs
    imgsz=640,  # Image resolution
    batch=16,  # Batch size (adjust depending on your GPU)
    name="entrenamiento_drones",  # Experiment name
    project="proyecto_drones"  # Folder to save results
)

# Check if the training output directory exists and list its contents
directory = "/content/proyecto_drones/entrenamiento_drones"
if os.path.exists(directory):
  print("Files in the directory:")
  print(os.listdir(directory))
else:
  print(f"The directory {directory} does not exist.")

# Load the training metrics into a pandas DataFrame
df_metrics = pd.read_csv("/content/proyecto_drones/entrenamiento_drones/results.csv")

# Display sample metrics
print(df_metrics.head(5))  # First 5 rows
print(df_metrics.index.to_list())  # Index values
print(df_metrics.columns.to_list())  # Column names

df_metrics.describe()  # Statistical summary of metrics

# Plot training metrics
plt.figure(figsize=(12, 6))
plt.plot(df_metrics[['epoch']], df_metrics[['train/cls_loss']], label='Train Loss')
plt.plot(df_metrics[['epoch']], df_metrics[['metrics/mAP50(B)']], label='Val mAP@0.5')
plt.plot(df_metrics[['epoch']], df_metrics[['metrics/mAP50-95(B)']], label='Val mAP@0.5:0.95')
plt.plot(df_metrics[['epoch']], df_metrics[['metrics/precision(B)']], label='Validation Precision')
plt.xlabel('Epoch')
plt.ylabel('Value')
plt.title('Training Metrics')
plt.legend()
plt.show()

# Validate the trained model
results = model.val()
print(results)

# Load the trained model for inference
modelo_entrenado = YOLO("/content/proyecto_drones/entrenamiento_drones/weights/best.pt")

# Run inference on a sample image
resultados = modelo_entrenado.predict(source="/content/377.jpg", save=True)

# Display prediction results: bounding boxes, confidence, and class
for resultado in resultados:
  print(f"Predictions for image: {resultado.path}")
  for box, conf, cls in zip(resultado.boxes.xyxy, resultado.boxes.conf, resultado.boxes.cls):
    print(f"Bounding Box: {box} - Confidence: {conf} - Class: {cls}")

# Load and display the image with predictions
img = Image.open("/content/runs/detect/predict3/377.jpg")
plt.imshow(img)
plt.axis('off')  # Hide axes
plt.show()

# Compress the project folder and download it
import shutil
from google.colab import files

folder_path = "/content/proyecto_drones"  # Folder to compress
output_filename = "Entrenar_IA_Yolo11.zip"  # Output zip file name

# Create the zip archive
shutil.make_archive(output_filename.replace('.zip', ''), 'zip', folder_path)

# Download the archive to your local machine
files.download(output_filename)
