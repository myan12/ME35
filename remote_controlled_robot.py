import bluetooth
import struct
import time
from machine import Pin, PWM, I2C
from neopixel import NeoPixel
from micropython import const

# must match TARGET_NAME on the controller
BLE_NAME = "MaggieBot"

# motor pins
LEFT_MOTOR = (12, 13)
RIGHT_MOTOR = (14, 27)
# if a wheel spins backwards
LEFT_INVERT = False
RIGHT_INVERT = True
# speed multiplier per wheel to correct drift
LEFT_TRIM = 1.0
RIGHT_TRIM = 1.0
MAX_SPEED = 100

# color sensor pins, each sensor on its own i2c bus since they share an address
LEFT_SDA, LEFT_SCL = 21, 22
RIGHT_SDA, RIGHT_SCL = 26, 25

# neopixels and buzzer
NEOPIXEL_PIN = 15
BUZZER_PIN = 23
# fanfare notes, one per flag swing
FANFARE = (523, 659, 784, 1047, 784, 1047, 1319, 1047)
NOTE_MS = 180

# flag servo and wave settings
SERVO_PIN = 4
FLAG_REST = 0
FLAG_LOW, FLAG_HIGH = 45, 135
FLAG_WAVES = 4
FLAG_STEP_MS = 250

# red tape detection (calibrated: floor ~0.44, red ~0.51)
RED_FRACTION = 0.44
MIN_BRIGHTNESS = 200
# how many red readings in a row before it counts as the finish line
RED_HITS_NEEDED = 3
SENSE_PERIOD_MS = 50

# stops motors if controller hasn't sent anything in this long
FAILSAFE_MS = 500
# prints sensor values with motors off, used for calibrating red
CALIBRATE = True


# motor class
class Motor:
    def __init__(self, pin_a, pin_b, invert=False):
        self.a = PWM(Pin(pin_a), freq=10000, duty_u16=0)
        self.b = PWM(Pin(pin_b), freq=10000, duty_u16=0)
        self.invert = invert

    # speed from -100 to 100 percent
    def set(self, speed):
        speed = max(-100, min(100, speed))
        if self.invert:
            speed = -speed
        duty = int(abs(speed) * 65535 / 100)
        if speed > 0:
            self.a.duty_u16(duty); self.b.duty_u16(0)
        elif speed < 0:
            self.a.duty_u16(0); self.b.duty_u16(duty)
        else:
            self.a.duty_u16(0); self.b.duty_u16(0)


left_motor = Motor(*LEFT_MOTOR, invert=LEFT_INVERT)
right_motor = Motor(*RIGHT_MOTOR, invert=RIGHT_INVERT)


def drive(l, r):
    left_motor.set(l * LEFT_TRIM)
    right_motor.set(r * RIGHT_TRIM)


def stop():
    drive(0, 0)


# converts joystick x (turning) and y (forward/back) into left and right wheel speeds
def arcade_drive(x, y):
    left = y + x
    right = y - x
    biggest = max(abs(left), abs(right), 100)
    return left * MAX_SPEED // biggest, right * MAX_SPEED // biggest


# neopixels
pixels = NeoPixel(Pin(NEOPIXEL_PIN), 2)


def leds(r, g, b):
    pixels[0] = (r, g, b)
    pixels[1] = (r, g, b)
    pixels.write()


# servo setup
servo = PWM(Pin(SERVO_PIN), freq=50, duty_u16=0)


# moves the servo to 0-180 degrees
def servo_angle(deg):
    deg = max(0, min(180, deg))
    us = 500 + deg * 2000 / 180
    servo.duty_u16(int(us * 65535 / 20000))


# puts the flag down, then stops the pulses so the servo doesn't jitter
def servo_rest():
    servo_angle(FLAG_REST)
    time.sleep_ms(400)
    servo.duty_u16(0)


# buzzer
buzzer = PWM(Pin(BUZZER_PIN), freq=1000, duty_u16=0)


# one flag swing with one fanfare note, so the music and the wave stay in sync
def swing(angle, note):
    servo_angle(angle)
    if note:
        buzzer.freq(note)
        buzzer.duty_u16(32768)
        time.sleep_ms(min(NOTE_MS, FLAG_STEP_MS))
        buzzer.duty_u16(0)
        time.sleep_ms(max(0, FLAG_STEP_MS - NOTE_MS))
    else:
        time.sleep_ms(FLAG_STEP_MS)


# finish line celebration: stop, red lights, flag wave with fanfare
# (same wave as the red maneuver from the line follower)
def celebrate():
    stop()
    leds(60, 0, 0)
    i = 0
    for _ in range(FLAG_WAVES):
        swing(FLAG_HIGH, FANFARE[i % len(FANFARE)] if i < len(FANFARE) else 0)
        i += 1
        swing(FLAG_LOW, FANFARE[i % len(FANFARE)] if i < len(FANFARE) else 0)
        i += 1
    buzzer.duty_u16(0)
    servo_rest()
    leds(0, 0, 0)


# color sensor class using i2c
class VEML6040:
    ADDR = 0x10

    # initializes color sensor to i2c bus (40 ms readings)
    def __init__(self, i2c):
        self.i2c = i2c
        self.i2c.writeto_mem(self.ADDR, 0x00, b"\x00\x00")

    # returns raw [R, G, B, W] values
    def read(self):
        out = []
        for reg in (0x08, 0x09, 0x0A, 0x0B):
            d = self.i2c.readfrom_mem(self.ADDR, reg, 2)
            out.append(d[0] | (d[1] << 8))
        return out


# how much of the color is red (0 to 1), ignoring readings that are too dark
def red_fraction(rgbw):
    r, g, b, _ = rgbw
    total = r + g + b
    if total <= MIN_BRIGHTNESS:
        return 0.0
    return r / total


# bluetooth events
_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE = const(3)

_FLAG_READ = const(0x0002)
_FLAG_WRITE_NO_RESPONSE = const(0x0004)
_FLAG_WRITE = const(0x0008)
_FLAG_NOTIFY = const(0x0010)

# channel ids, must match the controller
# TX = car sends messages to controller, RX = controller sends joystick data to car
_UART_UUID = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_TX = (bluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"), _FLAG_READ | _FLAG_NOTIFY)
_RX = (bluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E"), _FLAG_WRITE | _FLAG_WRITE_NO_RESPONSE)
_UART_SERVICE = (_UART_UUID, (_TX, _RX))


# builds the packet the car broadcasts so the controller can find it by name
def adv_payload(name):
    n = name.encode()
    return bytes([2, 0x01, 0x06, len(n) + 1, 0x09]) + n


# bluetooth class for the car
class BLERobot:
    def __init__(self, name):
        self.ble = bluetooth.BLE()
        self.ble.active(True)
        self.ble.irq(self._irq)
        ((self.tx, self.rx),) = self.ble.gatts_register_services((_UART_SERVICE,))
        self.conn = None
        # latest joystick packet (x, y, button) and when it arrived
        self.cmd = (0, 0, 0)
        self.cmd_ms = time.ticks_ms()
        self.payload = adv_payload(name)
        self.advertise()

    # broadcasts the car's name every 100 ms
    def advertise(self):
        self.ble.gap_advertise(100_000, adv_data=self.payload)
        print("Advertising as", BLE_NAME)

    # called by the bluetooth chip whenever something happens
    def _irq(self, event, data):
        if event == _IRQ_CENTRAL_CONNECT:
            self.conn = data[0]
            print("Controller connected")
        # lost connection: stop and start broadcasting again so it can reconnect
        elif event == _IRQ_CENTRAL_DISCONNECT:
            self.conn = None
            self.cmd = (0, 0, 0)
            print("Controller disconnected")
            self.advertise()
        # new joystick packet from the controller (20 times a second)
        elif event == _IRQ_GATTS_WRITE:
            conn, handle = data
            if handle == self.rx:
                msg = self.ble.gatts_read(self.rx)
                if len(msg) == 3:
                    self.cmd = struct.unpack("bbB", msg)
                    self.cmd_ms = time.ticks_ms()

    # sends a message back to the controller
    def send(self, data):
        if self.conn is not None:
            try:
                self.ble.gatts_notify(self.conn, self.tx, data)
            except OSError as e:
                print("Notify failed:", e)


# main
def run():
    # flag down at startup
    servo_rest()
    leds(0, 0, 0)

    robot = BLERobot(BLE_NAME)
    sens_l = VEML6040(I2C(0, sda=Pin(LEFT_SDA), scl=Pin(LEFT_SCL), freq=400_000))
    sens_r = VEML6040(I2C(1, sda=Pin(RIGHT_SDA), scl=Pin(RIGHT_SCL), freq=400_000))

    finished = False
    red_hits = 0
    last_btn = 0
    last_sense = time.ticks_ms()

    while True:
        now = time.ticks_ms()
        x, y, btn = robot.cmd

        # drive, but stop if disconnected or no packet recently (failsafe)
        stale = time.ticks_diff(now, robot.cmd_ms) > FAILSAFE_MS
        if CALIBRATE or robot.conn is None or stale:
            stop()
        else:
            l, r = arcade_drive(x, y)
            drive(l, r)

        # joystick press re-arms the finish line for another run
        if btn and not last_btn:
            finished = False
            red_hits = 0
            print("Finish detector re-armed")
        last_btn = btn

        # finish line detection, red on either sensor counts
        if time.ticks_diff(now, last_sense) >= SENSE_PERIOD_MS:
            last_sense = now
            raw_l = sens_l.read()
            raw_r = sens_r.read()
            fl = red_fraction(raw_l)
            fr = red_fraction(raw_r)
            is_red = fl > RED_FRACTION or fr > RED_FRACTION

            if CALIBRATE:
                print("L", raw_l, "red %.2f" % fl, "| R", raw_r, "red %.2f" % fr,
                      "RED" if is_red else "")

            red_hits = red_hits + 1 if is_red else 0
            # crossed the finish line: tell the controller first so both celebrate together
            if red_hits >= RED_HITS_NEEDED and not finished:
                finished = True
                robot.send(b"FINISH")
                print("Finish line crossed!")
                celebrate()
                # ignore joystick input from during the wave
                robot.cmd_ms = time.ticks_add(time.ticks_ms(), -FAILSAFE_MS - 1)

        time.sleep_ms(10)


# turn everything off if the program stops
try:
    run()
finally:
    stop()
    buzzer.duty_u16(0)
    servo.duty_u16(0)
    leds(0, 0, 0)