import cv2 as cv
import numpy as np
import paho.mqtt.client as mqtt
import base64
from PIL import Image as Img
from PIL import ImageTk
import tkinter as tk
from tkinter.simpledialog import askstring
import time
from dronekit import connect, VehicleMode
import threading


def on_message(client, userdata, message):
    if message.topic == 'picture':
        jpg_original = base64.b64decode(message.payload)
        jpg_as_np = np.frombuffer(jpg_original, dtype=np.uint8)
        image_buffer = cv.imdecode(jpg_as_np, 1)
        cv2image = cv.cvtColor(image_buffer, cv.COLOR_BGR2RGBA)
        img = Img.fromarray(cv2image)
        imgtk = ImageTk.PhotoImage(image=img)
        pictureLabel.imgtk = imgtk
        pictureLabel.configure(image=imgtk)

    if message.topic == 'status':
        statusLabel.config(text=str(message.payload.decode("utf-8")))

    if message.topic == 'mode':
        modeLabel.config(text=str(message.payload.decode("utf-8")))

    if message.topic == 'altitude':
        altitudeLabel.config(text=str(message.payload.decode("utf-8")))

    if message.topic == 'heading':
        headingLabel.config(text=str(message.payload.decode("utf-8")))


def getUpdateInfo():
    global client
    client.publish('updateInfo')


def ArmAndTakeoff():
    global client
    client.publish('arm&takeoff')


def startDetectColor():
    global client
    client.publish('startDetectColor')


def stopDetectColor():
    global client
    client.publish('stopDetectColor')


def goNorth():
    global client
    client.publish('north')


def goSouth():
    global client
    client.publish('south')


def goEast():
    global client
    client.publish('east')


def goWest():
    global client
    client.publish('west')


def prueba():
    global client
    client.publish('prueba')


master = tk.Tk()
# Frame info drone
autoPilotFrame = tk.LabelFrame(text="Autopilot control")
autoPilotFrame.grid(row=0, column=0, padx=5, pady=5)
getUpdateInfoButton = tk.Button(autoPilotFrame, text="Get drone info", bg='red', fg='white', command=getUpdateInfo)
getUpdateInfoButton.grid(row=0, column=0, padx=5, pady=5)
statusLabel = tk.Label(autoPilotFrame, text="status here")
statusLabel.grid(row=1, column=0, padx=5, pady=5)
headingLabel = tk.Label(autoPilotFrame, text="heading here")
headingLabel.grid(row=2, column=0, padx=5, pady=5)
modeLabel = tk.Label(autoPilotFrame, text="mode here")
modeLabel.grid(row=3, column=0, padx=5, pady=5)
altitudeLabel = tk.Label(autoPilotFrame, text="altitude here")
altitudeLabel.grid(row=4, column=0, padx=5, pady=5)
startDetectColorButton = tk.Button(autoPilotFrame, text="Start Detecting Colors", bg='red', fg='white', command=startDetectColor)
startDetectColorButton.grid(row=5, column=0, padx=5, pady=5)
startDetectColorButton = tk.Button(autoPilotFrame, text="Stop Detecting Colors", bg='red', fg='white', command=stopDetectColor)
startDetectColorButton.grid(row=6, column=0, padx=5, pady=5)
startMovingButton = tk.Button(autoPilotFrame, text="Go North", bg='red', fg='white', command=goNorth)
startMovingButton.grid(row=7, column=0, padx=5, pady=5)
startMovingButton = tk.Button(autoPilotFrame, text="Go South", bg='red', fg='white', command=goSouth)
startMovingButton.grid(row=8, column=0, padx=5, pady=5)
servoButton = tk.Button(autoPilotFrame, text="Go East", bg='red', fg='white', command=goEast)
servoButton.grid(row=9, column=0, padx=5, pady=5)
servoButton2 = tk.Button(autoPilotFrame, text="Go West", bg='red', fg='white', command=goWest)
servoButton2.grid(row=10, column=0, padx=5, pady=5)
servoButton3 = tk.Button(autoPilotFrame, text="Do it", bg='red', fg='white', command=prueba)
servoButton3.grid(row=11, column=0, padx=5, pady=5)
armButton = tk.Button(autoPilotFrame, text="Arm", bg='red', fg='white', command=ArmAndTakeoff)
armButton.grid(row=12, column=0, padx=5, pady=5)
# Frame camera view
cameraFrame = tk.LabelFrame(text="Camera control")
cameraFrame.grid(row=0, column=1, padx=5, pady=5)
pictureLabel = tk.Label(cameraFrame, text='Picture here')
pictureLabel.grid(row=0, column=0, columnspan=3)


broker_address = "192.168.1.115"  # "localhost" for laptop, "10.10.10.1" for raspi
broker_port = 1883

client = mqtt.Client("Little ground station")
client.on_message = on_message
client.connect(broker_address, broker_port)

client.loop_start()
client.subscribe('status')
client.subscribe('mode')
client.subscribe('altitude')
client.subscribe('heading')
client.subscribe('picture')

master.mainloop()