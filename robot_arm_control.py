from machine import Pin, PWM, disable_irq, enable_irq
import time

MOTOR1_PINS = (12, 13)    # motor driver inputs for DC motor 1
MOTOR2_PINS = (14, 27)    # motor driver inputs for DC motor 2
ENC1_A, ENC1_B = 32, 39   # encoder on DC motor 1  -> controls servo 1
ENC2_A, ENC2_B = 25, 33   # encoder on DC motor 2  -> controls servo 2
SERVO1_PIN = 4            # D4
SERVO2_PIN = 5            # D5

ENCODER_PPR = 11          # pulses per motor-shaft rev, per channel (11 is common for hall encoders)
GEAR_RATIO = 30           # e.g. 30 for a 30:1 gearmotor; use 1 if there's no gearbox
COUNTS_PER_REV = ENCODER_PPR * 4 * GEAR_RATIO   # counts per output-shaft rev
DEG_PER_REV = 180         # servo degrees per one full turn of the output shaft
LOOP_DELAY = 0.02         # 50 Hz update, matches the servo frame rate


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
        # Briefly block interrupts so the ISR can't change counter mid-write
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
    def __init__(self, pin, min_us=500, max_us=2500):
        self.pwm = PWM(Pin(pin), freq=50)
        self.min_us = min_us
        self.max_us = max_us

    def write(self, angle):
        us = self.min_us + (self.max_us - self.min_us) * angle / 180
        self.pwm.duty_u16(int(us * 65535 / 20000))   # 20000 us period at 50 Hz

    def off(self):
        self.pwm.duty_u16(0)
        self.pwm.deinit()


class EncoderServo(object):
    def __init__(self, motor, servo, start_angle=90, min_angle=0, max_angle=180, direction=1):
        self.motor = motor
        self.servo = servo
        self.start = start_angle
        self.min = min_angle
        self.max = max_angle
        self.k = direction * DEG_PER_REV / COUNTS_PER_REV   # degrees per count
        self.last = None
        self.motor.set_pos(0)
        self.servo.write(start_angle)

    def update(self):
        counts = self.motor.pos()
        angle = self.start + counts * self.k

        # Clamp, and pull the counter back to the limit so there's no "wind-up":
        # turning back immediately moves the servo instead of undoing extra turns first.
        if angle > self.max or angle < self.min:
            angle = max(self.min, min(self.max, angle))
            self.motor.set_pos(round((angle - self.start) / self.k))

        a = int(angle)
        if a != self.last:          # only rewrite PWM when the angle actually changes
            self.servo.write(a)
            self.last = a
            return True
        return False


Motor1 = Motor(MOTOR1_PINS[0], MOTOR1_PINS[1], ENC1_A, ENC1_B)
Motor2 = Motor(MOTOR2_PINS[0], MOTOR2_PINS[1], ENC2_A, ENC2_B)

joint1 = EncoderServo(Motor1, Servo(SERVO1_PIN))
joint2 = EncoderServo(Motor2, Servo(SERVO2_PIN))

try:
    while True:
        changed1 = joint1.update()
        changed2 = joint2.update()
        if changed1 or changed2:
            print("Motor1: {:6d} counts -> Servo1: {:3d} deg   Motor2: {:6d} counts -> Servo2: {:3d} deg".format(
                Motor1.pos(), joint1.last, Motor2.pos(), joint2.last))
        time.sleep(LOOP_DELAY)
except KeyboardInterrupt:
    Motor1.stop()
    Motor2.stop()
    joint1.servo.off()
    joint2.servo.off()
    print("Stopped")