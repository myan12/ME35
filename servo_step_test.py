from machine import Pin, PWM
import time

SERVO_PIN = 4
SERVO_FREQ = 50
SERVO_MIN_US = 500    # pulse width (us) at 0 degrees
SERVO_MAX_US = 2500   # pulse width (us) at 180 degrees

current_angle = 90

def angle_to_pulse_ns(angle):
    angle = max(0, min(180, angle))
    pulse_us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * (angle / 180)
    return int(pulse_us * 1000)

# Give the PWM object its starting duty right at construction, instead of
# letting it briefly output some default value first. This means there's
# only ONE snap on power-up (straight to current_angle's position) instead
# of an extra jump from the PWM's own default duty cycle. A servo has no
# feedback sensor, so it can never truly know or hold "wherever it happens
# to be" — this just avoids adding an unnecessary second jump on top of
# whatever the servo's actual physical resting angle was.
servo = PWM(Pin(SERVO_PIN), freq=SERVO_FREQ)
servo.duty_ns(angle_to_pulse_ns(current_angle))

def set_angle(angle):
    global current_angle
    angle = max(0, min(180, angle))
    servo.duty_ns(angle_to_pulse_ns(angle))
    current_angle = angle

def sweep_to(target_angle, step_delay_s=0.02):
    global current_angle
    step = 1 if target_angle > current_angle else -1
    for angle in range(current_angle, target_angle + step, step):
        set_angle(angle)
        time.sleep(step_delay_s)
    current_angle = target_angle

while True:
    print("stepping to 0")
    sweep_to(0)
    time.sleep(1)

    print("stepping to 90")
    sweep_to(45)
    time.sleep(1)

    print("stepping to 180")
    sweep_to(90)
    time.sleep(1)

    print("stepping to 90")
    sweep_to(135)
    time.sleep(1)
    
    print("stepping to 180")
    sweep_to(180)
    time.sleep(1)