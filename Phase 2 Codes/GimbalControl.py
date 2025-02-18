from pynput import keyboard
import pigpio

import time

# Raspberry GPIO
pi = pigpio.pi()

# GPIO servo pins
SERVO_YAW_PIN = 18
SERVO_PITCH_PIN = 13


# Valores iniciales de los servos
servo_pitch = 1500  # Valor neutro para el pitch
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

# Manejador de eventos de teclado
def on_press(key):
    try:
        if key == keyboard.Key.up:
            print("Pitch Up")
            move_pitch(STEP)
        elif key == keyboard.Key.down:
            print("Pitch Down")
            move_pitch(-STEP)
        elif key == keyboard.Key.left:
            print("Yaw Left")
            move_yaw(-STEP)
        elif key == keyboard.Key.right:
            print("Yaw Right")
            move_yaw(STEP)
    except AttributeError:
        pass

# Iniciar el listener de teclado
listener = keyboard.Listener(on_press=on_press)
listener.start()

# Bucle principal
try:
    print("Control del gimbal iniciado. Usa las flechas del teclado para controlar el gimbal.")
    while True:
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Saliendo...")
