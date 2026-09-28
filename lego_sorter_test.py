from machine import Pin, I2C
import time
import math
import veml6040


# ---------------------------------------------------------------------------
# Buttons
# ---------------------------------------------------------------------------

class Button:
    def __init__(self, pin_num, debounce_ms=50):
        self.pin = Pin(pin_num, Pin.IN, Pin.PULL_UP)
        self.debounce_ms = debounce_ms
        self.stable = self.pin.value()   # 1 = released, 0 = pressed
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


button_Mode = Button(25, debounce_ms=100)   # arcade buttons: slower/noisier contact
                                              # than the RoboESP32's built-in buttons,
                                              # so debounce is longer than the default
button_Color = Button(26, debounce_ms=100)


# ---------------------------------------------------------------------------
# Color sensor
# ---------------------------------------------------------------------------

i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)
print("i2c devices:", i2c.scan())

sensor = veml6040.VEML6040(i2c)
BIN_NAMES = {0: "red", 1: "green", 2: "blue", 3: "other"}


def sort_into_bin(bin_index):
    print("[TEST] would sort into bin", bin_index, "(", BIN_NAMES.get(bin_index), ")")


# ---------------------------------------------------------------------------
# Mode state machine
# ---------------------------------------------------------------------------

MODES = ["TRAIN_RED", "TRAIN_GREEN", "TRAIN_BLUE", "TRAIN_OTHER", "PLAY"]

# color label -> bin index, must match your physical bin layout
BIN_OF_COLOR = {"red": 0, "green": 1, "blue": 2, "other": 3}

mode_index = 0


def current_mode():
    return MODES[mode_index]


def mode_label():
    """The training label this mode corresponds to, or None if not a train mode."""
    mode = current_mode()
    if mode.startswith("TRAIN_"):
        return mode.split("_", 1)[1].lower()
    return None

data = []  # each entry: (r, g, b, label)


def k_nearest_neighbor(x, y, z, k=3):
    if not data:
        print("no training data yet")
        return None

    distances = []
    for d in data:
        dist = math.sqrt((x - d[0]) ** 2 + (y - d[1]) ** 2 + (z - d[2]) ** 2)
        distances.append((dist, d[3]))

    distances.sort(key=lambda p: p[0])
    nearest = distances[:min(k, len(distances))]
    classes = [label for _, label in nearest]

    print("k nearest classes:", classes)
    winner = max(set(classes), key=classes.count)
    print("classified as:", winner)
    return winner


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

print("starting in mode:", current_mode())

while True:
    if button_Mode.pressed():
        mode_index = (mode_index + 1) % len(MODES)
        print("mode ->", current_mode())

    if button_Color.pressed():
        red, green, blue, white = sensor.read_rgbw()
        mode = current_mode()

        if mode == "PLAY":
            predicted = k_nearest_neighbor(red, green, blue, k=3)
            if predicted is not None and predicted in BIN_OF_COLOR:
                bin_index = BIN_OF_COLOR[predicted]
                print("sorting into bin", bin_index, "(", predicted, ")")
                sort_into_bin(bin_index)
            else:
                print("could not classify sample; skipping sort")

        else:
            label = mode_label()
            data.append((red, green, blue, label))
            print("trained:", label, "->", (red, green, blue), "| total samples:", len(data))

    time.sleep(0.05)