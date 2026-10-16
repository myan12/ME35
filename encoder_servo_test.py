from machine import Pin, PWM, disable_irq, enable_irq
import time

# ---------------- Pin assignments (change to match your wiring) ----------------
MOTOR_PINS = (12, 13)     # motor driver inputs
ENC_A, ENC_B = 32, 39     # encoder on the DC motor
SERVO_PIN = 4             # D4

# ---------------- Tuning ----------------
# Encoder is on the motor shaft, so one turn of the OUTPUT shaft
# = ENCODER_PPR * 4 * GEAR_RATIO counts (x4 = every edge on A and B).
ENCODER_PPR = 11          # pulses per motor-shaft rev, per channel
GEAR_RATIO = 30           # use 1 if there's no gearbox
COUNTS_PER_REV = ENCODER_PPR * 4 * GEAR_RATIO
DEG_PER_REV = 180         # servo degrees per one full turn of the output shaft
START_ANGLE = 90          # servo angle when the program starts
MIN_ANGLE, MAX_ANGLE = 0, 180
DIRECTION = 1             # set to -1 if the servo moves the wrong way
LOOP_DELAY = 0.02


class Count(object):
    def __init__(self, A, B):
        self.A = Pin(A, Pin.IN)
        self.B = Pin(B, Pin.IN)
        self.counter = 0
        self.A.irq(self.cb, Pin.IRQ_FALLING | Pin.IRQ_RISING)
        self.B.irq(self.cb, Pin.IRQ_FALLING | Pin.IRQ_RISING)

    def cb(self, msg):
        other, inc = (self.B, 1) if msg == self.A else (self.A, -1)
        self.counter += -inc if msg.value() != other.value() else inc

    def value(self):
        return self.counter

    def set(self, value):
        state = disable_irq()
        self.counter = value
        enable_irq(state)


class Motor(Count):
    def __init__(self, m1, m2, A, B):
        self.enc = Count(A, B)
        self.M1 = PWM(m1, freq=50, duty_u16=0)
        self.M2 = PWM(m2, freq=50, duty_u16=0)
        self.stop()

    def pos(self):
        return self.enc.value()

    def set_pos(self, value):
        self.enc.set(value)

    def stop(self):
        self.M1.duty_u16(0)
        self.M2.duty_u16(0)

    def start(self, direction=0, speed=50):
        if direction:
            self.M1.duty_u16(int(speed * 65535 / 100))
            self.M2.duty_u16(0)
        else:
            self.M1.duty_u16(0)
            self.M2.duty_u16(int(speed * 65535 / 100))


class Servo(object):
    """50 Hz hobby servo, 500-2500 us pulse range."""
    def __init__(self, pin, min_us=500, max_us=2500):
        self.pwm = PWM(Pin(pin), freq=50)
        self.min_us = min_us
        self.max_us = max_us

    def write(self, angle):
        us = self.min_us + (self.max_us - self.min_us) * angle / 180
        self.pwm.duty_u16(int(us * 65535 / 20000))

    def off(self):
        self.pwm.duty_u16(0)
        self.pwm.deinit()


# ---------------- Main ----------------
motor = Motor(MOTOR_PINS[0], MOTOR_PINS[1], ENC_A, ENC_B)
servo = Servo(SERVO_PIN)

deg_per_count = DIRECTION * DEG_PER_REV / COUNTS_PER_REV
motor.set_pos(0)
servo.write(START_ANGLE)
last_angle = START_ANGLE

try:
    while True:
        counts = motor.pos()
        angle = START_ANGLE + counts * deg_per_count

        if angle > MAX_ANGLE or angle < MIN_ANGLE:
            angle = max(MIN_ANGLE, min(MAX_ANGLE, angle))
            motor.set_pos(round((angle - START_ANGLE) / deg_per_count))

        angle = int(angle)
        if angle != last_angle:
            servo.write(angle)
            last_angle = angle
            print("counts: {:6d}  ->  servo: {:3d} deg".format(motor.pos(), angle))

        time.sleep(LOOP_DELAY)
except KeyboardInterrupt:
    motor.stop()
    servo.off()
    print("Stopped")