import cv2 as cv
import paho.mqtt.client as mqtt
import base64
import time
from dronekit import connect, VehicleMode
import threading
import numpy as np
from pymavlink import mavutil
import math

broker_address = "192.168.1.115"
broker_port = 1883


def goNorth():
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0,
        0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,
        0,
        0,
        0,
        speed,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    vehicle.send_mavlink(msg)
    #time.sleep(1)


def goSouth():
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0,
        0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,
        0,
        0,
        0,
        -speed,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    vehicle.send_mavlink(msg)
    #time.sleep(10)


def goEast():
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0,
        0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,
        0,
        0,
        0,
        0,
        speed,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    vehicle.send_mavlink(msg)
    #time.sleep(10)


def goWest():
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0,
        0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,
        0,
        0,
        0,
        0,
        -speed,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    vehicle.send_mavlink(msg)
    #time.sleep(10)


def update_info():
    global updating
    status = ""
    mode = "Waiting to arm..."
    altitude = "Waiting to arm..."
    heading = "Waiting to arm..."
    while updating:
        if not vehicle.armed:
            status = "Not armed"
        if vehicle.armed:
            status = "Armed"
            mode = str(vehicle.mode.name)
            altitude = str(vehicle.location.global_relative_frame.alt)
            heading = str(vehicle.heading)
        if vehicle.mode.name == 'RTL':
            if not vehicle.armed:
                status = "Landed"

        client.publish('status', status)
        client.publish('mode', mode)
        client.publish('altitude', altitude)
        client.publish('heading', heading)
        time.sleep(1)


def arm_and_takeoff():
    status = "Basic Pre-Arm checks"

    while not vehicle.is_armable:
        status = "Basic Pre-Arm checks"
        time.sleep(1)

    status = "Arming motors"
    # Drone should be armed in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed
    while not vehicle.armed:
        status = "Waiting for arming"
        time.sleep(1)

    status = "Taking off!"
    client.publish('status', status)
    vehicle.simple_takeoff(6)
    time.sleep(5)
    status = "Ready to detect colors and moving"
    client.publish('status', status)


def detectColor():
    cap = cv.VideoCapture(0)
    while detectingColor:
        ret, frame = cap.read()
        _, buffer = cv.imencode('.jpg', frame)
        # Converting into encoded bytes for transmission
        jpg_as_txt = base64.b64encode(buffer)
        client.publish('picture', jpg_as_txt)

        time.sleep(0.25)


def RTL():
    vehicle.mode = VehicleMode("RTL")



def DistanceInMeters(loc1,loc2):
    dlat = loc1.lat - loc2.lat
    dlon = loc1.lon - loc2.lon
    return math.sqrt((dlat*dlat)+(dlon*dlon))*1.113195e5



def prueba():
    topsecret = True
    checkpoints = [0,0,0,0,0]
    n = 0
    while topsecret:
        goNorth()
        time.sleep(10)
        checkpoints[n] = vehicle.location.global_frame
        print(checkpoints)
        n = n + 1

        goEast()
        time.sleep(5)
        checkpoints[n] = vehicle.location.global_frame
        print(checkpoints)
        n = n + 1

        goSouth()
        time.sleep(10)
        checkpoints[n] = vehicle.location.global_frame
        print(checkpoints)

        goWest()
        time.sleep(5)

        #entrega
        time.sleep(1)
        currentlocation = vehicle.location.global_relative_frame
        currentlocation.alt = 3
        vehicle.simple_goto(currentlocation)
        time.sleep(6)
        currentlocation = vehicle.location.global_relative_frame
        currentlocation.alt = 6
        vehicle.simple_goto(currentlocation)
        time.sleep(6)

        time.sleep(1)
        i = 0
        while i <= n:
            vehicle.simple_goto(checkpoints[n-i])
            currentlocaton = vehicle.location.global_frame
            dist = DistanceInMeters(checkpoints[n-i],currentlocaton)
            #Wait until reach point
            while dist > 0.5:
                time.sleep(0.25)
                currentlocaton = vehicle.location.global_frame
                dist = DistanceInMeters(checkpoints[n-i],currentlocaton)
            time.sleep(1)
            i = i +1

        RTL()

        topsecret = False




def on_message(client, userdata, message):
    global updating
    global detectingColor
    global moving

    if message.topic == 'updateInfo':
        print('update info')
        updating = True
        update = threading.Thread(target=update_info)
        update.start()

    if message.topic == 'arm&takeoff':
        print('arm and takeoff')
        arm_and_takeoff()

    if message.topic == 'startDetectColor':
        print('start detect color')
        detectingColor = True
        color = threading.Thread(target=detectColor)
        color.start()

    if message.topic == 'stopDetectColor':
        print('stop detect color')
        detectingColor = False

    if message.topic == 'north':
        print('going north')
        goNorth()
    if message.topic == 'south':
        print('going south')
        goSouth()
    if message.topic == 'east':
        print('going east')
        goEast()
    if message.topic == 'west':
        print('going west')
        goWest()
    if message.topic == 'prueba':
        print('Executing order 66')
        prueba()


detectingColor = False
moving = False
updating = False

xI, yI, xF, yF = 295, 215, 355, 265

speed = 1

connection_string = "tcp:127.0.0.1:5762" # 'tcp:127.0.0.1:5762' for MP sim, "/dev/ttyS0" for raspi
vehicle = connect(connection_string, wait_ready=True, baud=115200)

client = mqtt.Client("On board controller")
client.on_message = on_message
client.connect(broker_address, broker_port)
client.subscribe('updateInfo')
client.subscribe('arm&takeoff')
client.subscribe('startDetectColor')
client.subscribe('stopDetectColor')
client.subscribe('north')
client.subscribe('south')
client.subscribe('east')
client.subscribe('west')
client.subscribe('prueba')
client.loop_forever()