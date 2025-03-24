from __future__ import print_function
from dronekit import connect, VehicleMode
import time
import tkinter as tk
from pymavlink import mavutil
import math
import numpy as np
import cv2

def colorDetector(img):
    color = "None"
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # define range of red color in HSV
    lower_red = np.array([50, 50, 110])  # min HSV values of red
    upper_red = np.array([255, 255, 130])  # max HSV values of red
    # Threshold the HSV image to get only red colors
    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    print(mask_red.mean())
    if mask_red.mean() > 50:
        color = "RED"

    # define range of blue color in HSV
    lower_blue = np.array([110,50,50])  # min HSV values of blue
    upper_blue = np.array([130, 255, 255])  # max HSV values of blue
    # Threshold the HSV image to get only blue colors
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    print(mask_blue.mean())
    if mask_blue.mean() > 50:
        color = "BLUE"

    # define range of yellow color in HSV
    lower_yellow = np.array([0,100,100])  # min HSV values of yellow
    upper_yellow = np.array([70,255,255])  # max HSV values of yellow
    # Threshold the HSV image to get only yellow colors
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    print(mask_yellow.mean())
    if mask_yellow.mean() > 50:
        color = "YELLOW"


    #funcion q vea si tiene blanco la mascara
    cv2.imshow("mask_red", mask_red)
    cv2.imshow("mask_blue", mask_blue)
    cv2.imshow("mask_yellow", mask_yellow)



    return(color)


# Crea las dimensiones del cuadrado pequeño central a recortar
xI,yI,xF,yF=275,195,375,285
# Obtiene la imagen
frame = cv2.imread('ImageAmarillo.jpg')
# Recorta la imagen
img=frame[yI:yF,xI:xF,:]
# Muestra las imagenes
cv2.imshow('img',img)
cv2.imshow('img sin recorte',frame)
# Aplica la funcion
color = colorDetector(img)
print(color)
cv2.waitKey(0)


