import asyncio
import websockets
import cv2
import base64
import pigpio
import time
from picamera2 import Picamera2, Preview

# Server setters
video_clients = set()
command_clients = set()

# Camera init.
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")

#GPIO config
pi = pigpio.pi()
SERVO_YAW_PIN = 18 #Servo pins
SERVO_PITCH_PIN = 13
# Valores iniciales de los servos
servo_pitch = 1850  # Valor neutro para el pitch
servo_yaw = 1500  # Valor neutro para el yaw
# Rango de valores de PWM para los servos
PWM_MIN = 500
PWM_MAX = 2500
STEP = 50  # Incremento o decremento en cada tecla

def update_servo(servo_pin, servo_value):
    pi.set_servo_pulsewidth(servo_pin, servo_value)
    width = pi.get_servo_pulsewidth(servo_pin)
    
    print(f'New servo pulse width: {width}')

def move_pitch(delta):
    global servo_pitch
    servo_pitch = max(PWM_MIN, min(PWM_MAX, servo_pitch + delta))
    print(f'Moving pitch to: {servo_pitch}')
    update_servo(SERVO_PITCH_PIN, servo_pitch)  # AUX2 suele estar en el canal 10

def move_yaw(delta):
    global servo_yaw
    servo_yaw = max(PWM_MIN, min(PWM_MAX, servo_yaw + delta))
    print(f'Moving yaw to: {servo_yaw}')
    update_servo(SERVO_YAW_PIN, servo_yaw)  # AUX2 suele estar en el canal 10

def neutral_gimbal():
    global servo_pitch, servo_yaw
    servo_pitch = 1850  # Valor neutro para el pitch
    servo_yaw = 1500  # Valor neutro para el yaw
    print(f'Moving gimbal to neutral position')
    update_servo(SERVO_YAW_PIN, servo_yaw)
    update_servo(SERVO_PITCH_PIN, servo_pitch)

async def process_video(websocket, path=None):
    video_clients.add(websocket)
    try:
        print("Video client connected")

        frame_count = 0

        while True:
            frame = picam2.capture_array()

            frame_count += 1
            print(f"Processing frame {frame_count}...")

            #gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            _, frame_data = cv2.imencode('.jpg', frame)
            base64_frame = base64.b64encode(frame_data).decode("utf-8")

            await websocket.send(base64_frame)
            await asyncio.sleep(0.1)

        cap.release()
        print("Video processing complete. Closing connection.")
        await websocket.close()

    except websockets.exceptions.ConnectionClosedError:
        print("Video client disconnected. Waiting for a new connection...")
    except Exception as e:
        print(f"Error on the video server: {str(e)}")
    finally:
        video_clients.remove(websocket)

async def process_commands(websocket, path=None):
    command_clients.add(websocket)
    try:
        print("Command client connected")
        while True:
            command = await websocket.recv()
            print(f"Received command: {command}")
            try:
                if command == "up":
                    print("Pitch Up")
                    move_pitch(-STEP)
                elif command == "down":
                    print("Pitch Down")
                    move_pitch(STEP)
                elif command == "left":
                    print("Yaw Left")
                    move_yaw(STEP)
                elif command == "right":
                    print("Yaw Right")
                    move_yaw(-STEP)
                elif command=="neutral":
                    neutral_gimbal()
                    
            except AttributeError:
                pass

    except websockets.exceptions.ConnectionClosedError:
        print("Command client disconnected. Waiting for a new connection...")
    except Exception as e:
        print(f"Error on the command server: {str(e)}")
    finally:
        command_clients.remove(websocket)

async def main():
    
    picam2.start()
    video_server = websockets.serve(process_video, "0.0.0.0", 8765)
    command_server = websockets.serve(process_commands, "0.0.0.0", 8766)
    await asyncio.gather(video_server, command_server)
    print("WebSocket servers started.")
    await asyncio.Future()

if __name__ == "__main__":
    import sys

    if sys.platform.startswith("win"):
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())

    asyncio.run(main())
