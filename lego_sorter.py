from machine import Pin, I2C, PWM
import time
import math
import veml6040

try:
    import ujson as json
except ImportError:
    import json

# class for arcade button
class Button:
    def __init__(self, pin_num, debounce_ms=50):
        self.pin = Pin(pin_num, Pin.IN, Pin.PULL_UP)
        self.debounce_ms = debounce_ms
        self.stable = self.pin.value()
        self.last_raw = self.stable
        self.last_change = time.ticks_ms()

    def pressed(self):
        raw = self.pin.value()
        now = time.ticks_ms()
        if raw != self.last_raw:
            self.last_raw = raw
            self.last_change = now
        if time.ticks_diff(now, self.last_change) > self.debounce_ms and raw != self.stable:
            self.stable = raw
            return raw == 0
        return False

# adjusted debounce time for arcade button speed
button_Mode = Button(25, debounce_ms=50)
button_Color = Button(26, debounce_ms=50)

# hardware I2C using corresponding pins
i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)
print("i2c devices:", i2c.scan())

sensor = veml6040.VEML6040(i2c)

def normalize_rgb(r, g, b):
    total = r + g + b
    if total == 0:
        return (0.0, 0.0, 0.0)
    return (r / total, g / total, b / total)

# servo globals
SERVO_FREQ = 50
SERVO_MIN_US = 500   # pulse width (us) at 0 degrees
SERVO_MAX_US = 2500  # pulse width (us) at 180 degrees

# class for servos
class Servo:
    def __init__(self, pin_num):
        self.pwm = PWM(Pin(pin_num), freq=SERVO_FREQ)
        self.current_angle = 0

    def set_angle(self, angle):
        angle = max(0, min(180, angle))
        pulse_us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * (angle / 180)
        self.pwm.duty_ns(int(pulse_us * 1000))
        self.current_angle = angle

    def sweep_to(self, target_angle, step_delay_s=0.02):
        target_angle = max(0, min(180, target_angle))
        step = 1 if target_angle > self.current_angle else -1
        for angle in range(self.current_angle, target_angle + step, step):
            self.set_angle(angle)
            time.sleep(step_delay_s)

# servo motor to convert rotational to linear motion
# CALIBRATE THIS
RACK_SERVO_PIN = 4
RACK_DETECT_ANGLE = 90
BIN_ANGLES = {
    0: 0,    # bin 0 (red)
    1: 60,    # bin 1 (green)
    2: 120,   # bin 2 (blue)
    3: 145,   # bin 3 (other)
}

# servo to flick lego into bin
FLICK_SERVO_PIN = 5
FLICK_REST = 15     # CALIBRATE
FLICK_FIRE = 90    # CALIBRATE
FLICK_SETTLE_S = 0.3  # time to let the servo actually get there before moving on

# init servos
rack_servo = Servo(RACK_SERVO_PIN)
rack_servo.sweep_to(RACK_DETECT_ANGLE)  # start at the detect position
flick_servo = Servo(FLICK_SERVO_PIN)
flick_servo.sweep_to(FLICK_REST)

# servo motor movement based on bin index
def sort_into_bin(bin_index):
    rack_servo.sweep_to(BIN_ANGLES[bin_index])   # gradual move, not jerky
    time.sleep(FLICK_SETTLE_S)
    flick_servo.sweep_to(FLICK_FIRE, step_delay_s=0.01)
    time.sleep(FLICK_SETTLE_S)
    flick_servo.sweep_to(FLICK_REST, step_delay_s=0.01)
    time.sleep(FLICK_SETTLE_S)
    rack_servo.sweep_to(RACK_DETECT_ANGLE)
    time.sleep(2)

# state machine for modes using array of states
MODES = ["TRAIN_RED", "TRAIN_GREEN", "TRAIN_BLUE", "TRAIN_YELLOW", "TRAIN_BLACK", "PLAY"]
BIN_OF_COLOR = {"red": 0, "green": 1, "blue": 2, "yellow": 3, "black": 3, "not sure": 3}
mode_index = 0

# function returns current mode based on index
def current_mode():
    return MODES[mode_index]

# function returns which color is being trained
# returns none if not in training mode
def mode_label():
    mode = current_mode()
    if mode.startswith("TRAIN_"):
        return mode.split("_", 1)[1].lower()
    return None

data = []  # each entry: (r, g, b, label)
MAX_DISTANCE = 0.05
DATA_FILE = "training_data.json"

def load_data():
    global data
    try:
        with open(DATA_FILE) as f:
            data = json.load(f)
        print("loaded", len(data), "training samples from", DATA_FILE)
    except OSError:
        data = []
        print("no existing training data found at", DATA_FILE, "- starting fresh")

def save_data():
    with open(DATA_FILE, "w") as f:
        json.dump(data, f)
    print("saved", len(data), "training samples to", DATA_FILE)

load_data()

def k_nearest_neighbor(x, y, z, k=3, max_distance=MAX_DISTANCE):
    if not data:
        print("no training data yet")
        return None

    distances = []
    for d in data:
        dist = math.sqrt((x - d[0]) ** 2 + (y - d[1]) ** 2 + (z - d[2]) ** 2)
        distances.append((dist, d[3]))

    distances.sort(key=lambda p: p[0])
    closest_dist = distances[0][0]
    print("closest_dist:", closest_dist)
    if closest_dist > max_distance:
        print("closest match is too far (", closest_dist, ">", max_distance, ") -> unsure")
        return "unsure"

    nearest = distances[:min(k, len(distances))]
    classes = [label for _, label in nearest]

    print("k nearest classes:", classes)
    winner = max(set(classes), key=classes.count)
    print("classified as:", winner)
    return winner

print("starting in mode:", current_mode())

while True:
    # if the mode button is pressed return current mode
    if button_Mode.pressed():
        mode_index = (mode_index + 1) % len(MODES)
        print("mode ->", current_mode())

    # if the color button is pressed, read the current color
    if button_Color.pressed():
        rack_servo.sweep_to(RACK_DETECT_ANGLE)
        red, green, blue, white = sensor.read_rgbw()
        r_norm, g_norm, b_norm = normalize_rgb(red, green, blue)
        mode = current_mode()

        # if current mode is in play
        # classify the nearest neighbor and sort into bin based on predicted color
        if mode == "PLAY":
            predicted = k_nearest_neighbor(r_norm, g_norm, b_norm, k=3)
            if predicted is not None and predicted in BIN_OF_COLOR:
                bin_index = BIN_OF_COLOR[predicted]
                print("sorting into bin", bin_index, "(", predicted, ")")
                sort_into_bin(bin_index)
            else:
                print("could not classify sample; skipping sort")
        # if in training mode
        # label the data and append to current data samples
        else:
            label = mode_label()
            data.append((r_norm, g_norm, b_norm, label))
            save_data()
            print("trained:", label, "->", (red, green, blue), "| total samples:", len(data))

    time.sleep(0.05)