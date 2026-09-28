from machine import Pin, PWM
import time

SERVO_PIN = 5
SERVO_FREQ = 50
SERVO_MIN_US = 500    # pulse width (us) at 0 degrees
SERVO_MAX_US = 2500   # pulse width (us) at 180 degrees

current_angle = 0  # starting angle, matches physical start position

def angle_to_pulse_ns(angle):
    angle = max(0, min(180, angle))
    pulse_us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * (angle / 180)
    return int(pulse_us * 1000)

servo = PWM(Pin(SERVO_PIN), freq=SERVO_FREQ)
servo2 = PWM(Pin(4), freq = SERVO_FREQ)
servo2.deinit()
servo.duty_ns(angle_to_pulse_ns(current_angle))

def set_angle(angle):
    global current_angle
    angle = max(0, min(180, angle))
    servo.duty_ns(angle_to_pulse_ns(angle))
    current_angle = angle

def sweep_to(target_angle, step_delay_s=0.01):
    global current_angle
    step = 1 if target_angle > current_angle else -1
    for angle in range(current_angle, target_angle + step, step):
        set_angle(angle)
        time.sleep(step_delay_s)
    current_angle = target_angle

while True:
    print("flicking to 90")
    sweep_to(90)
    time.sleep(1)

    print("returning to 0")
    sweep_to(0)
    time.sleep(1)