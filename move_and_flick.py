from machine import Pin, PWM
import time

SERVO_FREQ = 50
SERVO_MIN_US = 500    # pulse width (us) at 0 degrees
SERVO_MAX_US = 2500   # pulse width (us) at 180 degrees

RACK_SERVO_PIN = 4
FLICK_SERVO_PIN = 5

RACK_START_ANGLE = 90
FLICK_REST_ANGLE = 0
FLICK_FIRE_ANGLE = 90

RACK_POSITIONS = [0, 90, 135, 180]  # every rack position to test, in order
PAUSE_BETWEEN_S = 5


class Servo:
    def __init__(self, pin_num, start_angle):
        self.current_angle = start_angle
        self.pwm = PWM(Pin(pin_num), freq=SERVO_FREQ)
        # set the starting duty immediately, so there's only one snap on
        # power-up (straight to start_angle) instead of an extra jump from
        # the PWM's own default duty cycle
        self.pwm.duty_ns(self._angle_to_pulse_ns(start_angle))

    def _angle_to_pulse_ns(self, angle):
        angle = max(0, min(180, angle))
        pulse_us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * (angle / 180)
        return int(pulse_us * 1000)

    def set_angle(self, angle):
        angle = max(0, min(180, angle))
        self.pwm.duty_ns(self._angle_to_pulse_ns(angle))
        self.current_angle = angle

    def sweep_to(self, target_angle, step_delay_s=0.02):
        """Move gradually to target_angle, one degree at a time."""
        target_angle = max(0, min(180, target_angle))
        step = 1 if target_angle > self.current_angle else -1
        for angle in range(self.current_angle, target_angle + step, step):
            self.set_angle(angle)
            time.sleep(step_delay_s)


rack_servo = Servo(RACK_SERVO_PIN, RACK_START_ANGLE)
flick_servo = Servo(FLICK_SERVO_PIN, FLICK_REST_ANGLE)

def flick():
    print("  flicking")
    flick_servo.sweep_to(FLICK_FIRE_ANGLE, step_delay_s = 0.01)
    flick_servo.sweep_to(FLICK_REST_ANGLE, step_delay_s = 0.01)


while True:
    for position in RACK_POSITIONS:
        print("rack -> ", position)
        rack_servo.sweep_to(position)
        flick()
        print("  waiting", PAUSE_BETWEEN_S, "s")
        time.sleep(PAUSE_BETWEEN_S)