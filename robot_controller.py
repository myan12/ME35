import bluetooth
import struct
import time
from machine import Pin, ADC, PWM
from neopixel import NeoPixel
from micropython import const

TARGET_NAME = "MaggieBot"

# joystick pins
JOY_X_PIN = 36
JOY_Y_PIN = 39
BUTTON_PINS = (34, 35)

# joystick calibration values for left/right/up/down
JOY_PRESS_LEVEL = 0.97
JOY_LOW = 0.21
JOY_HIGH = 0.79
# ignore small changes in joystick to maintain straight driving
DEADZONE = 0.08

# tuning left/right turning speeds (as fraction of full speed)
# slightly offset for even speed
TURN_SCALE_LEFT = 0.45
TURN_SCALE_RIGHT = 0.75
TURN_EXPO = True

# if joystick is inverted
INVERT_X = False
INVERT_Y = False

# buzzer and neopixels for celebrating at the finish line
BUZZER_PIN = 23
NEOPIXEL_PIN = 15
NUM_PIXELS = 2
BRIGHTNESS = 60                 # 0-255

SEND_PERIOD_MS = 50

# joystick class
class Joystick:
    """Class analog joystick on D36/D39. read() returns (x, y, pressed)
    with x, y in -100..100 and pressed = 0/1."""

    def __init__(self):
        self.ax = ADC(Pin(JOY_X_PIN))
        self.ay = ADC(Pin(JOY_Y_PIN))
        self.ax.atten(ADC.ATTN_11DB)   # full 0-3.3 V range
        self.ay.atten(ADC.ATTN_11DB)
        self.buttons = [Pin(p, Pin.IN) for p in BUTTON_PINS]
        # calibrate center: DON'T touch the stick while the board boots
        self.cx = self._avg(self.ax)
        self.cy = self._avg(self.ay)
        print("Joystick center: x=%.2f y=%.2f" % (self.cx, self.cy))

    @staticmethod
    def _avg(adc, n=32):
        return sum(adc.read_u16() for _ in range(n)) / n / 65535

    @staticmethod
    def _norm(v, center):
        if v >= center:
            out = (v - center) / max(0.01, JOY_HIGH - center)
        else:
            out = (v - center) / max(0.01, center - JOY_LOW)
        out = max(-1.0, min(1.0, out))
        if abs(out) < DEADZONE:
            return 0
        # rescale so speed ramps up smoothly from the edge of the deadzone
        sign = 1 if out > 0 else -1
        return int(sign * (abs(out) - DEADZONE) / (1 - DEADZONE) * 100)

    def read(self):
        x_raw = self.ax.read_u16() / 65535
        y_raw = self.ay.read_u16() / 65535

        # press = X slammed to full scale (stick pushed in), or an onboard button
        pressed = x_raw > JOY_PRESS_LEVEL or any(b.value() == 0 for b in self.buttons)

        x = 0 if pressed else self._norm(x_raw, self.cx)   # don't steer hard while clicking
        y = self._norm(y_raw, self.cy)

        if TURN_EXPO:
            x = x * abs(x) // 100
        if INVERT_X:
            x = -x                      # flip first so negative always means a left turn
        x = int(x * (TURN_SCALE_LEFT if x < 0 else TURN_SCALE_RIGHT))
        if INVERT_X:
            x = -x
        if INVERT_Y:
            y = -y
        return x, y, 1 if pressed else 0

# buzzer and neopixel setup
buzzer = PWM(Pin(BUZZER_PIN), freq=1000, duty_u16=0)
pixels = NeoPixel(Pin(NEOPIXEL_PIN), NUM_PIXELS)

B = BRIGHTNESS
OFF = (0, 0, 0)
RAINBOW = ((B, 0, 0), (B, B // 2, 0), (B, B, 0), (0, B, 0), (0, B, B), (0, 0, B), (B // 2, 0, B))
# status colors: dim green = connected, blinking blue = searching for the car
CONNECTED = (0, 12, 0)
SEARCHING = (0, 0, 25)

# ta-da tune from the desktop pet: (frequency Hz, length ms)
TADA = ((784, 120), (988, 120), (1175, 120), (1568, 300))


def leds(color, color2=None):
    # sets both pixels, or each one separately if color2 is given
    pixels[0] = color
    pixels[1 % NUM_PIXELS] = color if color2 is None else color2
    pixels.write()


def celebrate():
    # ta-da on the buzzer while the pixels step through a rainbow, then a sparkle (~1.5 s)
    for i, (f, ms) in enumerate(TADA):
        leds(RAINBOW[(2 * i) % len(RAINBOW)], RAINBOW[(2 * i + 1) % len(RAINBOW)])
        buzzer.freq(f)
        buzzer.duty_u16(20000)
        time.sleep_ms(ms)
        buzzer.duty_u16(0)
        time.sleep_ms(30)
    # sparkle: the two pixels swap colors
    for i in range(10):
        a, b = RAINBOW[i % len(RAINBOW)], RAINBOW[(i + 3) % len(RAINBOW)]
        leds(a, b) if i % 2 else leds(b, a)
        time.sleep_ms(80)
    leds(OFF)

_IRQ_SCAN_RESULT = const(5)
_IRQ_SCAN_DONE = const(6)
_IRQ_PERIPHERAL_CONNECT = const(7)
_IRQ_PERIPHERAL_DISCONNECT = const(8)
_IRQ_GATTC_SERVICE_RESULT = const(9)
_IRQ_GATTC_SERVICE_DONE = const(10)
_IRQ_GATTC_CHARACTERISTIC_RESULT = const(11)
_IRQ_GATTC_CHARACTERISTIC_DONE = const(12)
_IRQ_GATTC_NOTIFY = const(18)

_UART_UUID = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_RX_UUID = bluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E")
_TX_UUID = bluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E")


def adv_name(adv):
    i = 0
    while i + 1 < len(adv):
        length = adv[i]
        if length == 0:
            break
        if adv[i + 1] in (0x08, 0x09):
            try:
                return bytes(adv[i + 2:i + 1 + length]).decode()
            except Exception:
                return None
        i += 1 + length
    return None


class BLEController:
    # initializes bluetooth on ESP32 and calls interrupt handler every time an input happens
    def __init__(self, target, on_message):
        self.target = target
        self.on_message = on_message
        self.ble = bluetooth.BLE()
        self.ble.active(True)
        self.ble.irq(self._irq)
        self.need_scan = False
        self._reset()

    # reset upon connection (erases previous info)
    def _reset(self):
        self.conn = None
        self.addr = None
        self.addr_type = None
        self.svc_start = None
        self.svc_end = None
        self.rx = None
        self.tx = None
        self.ready = False

    # scans for nearby bluetooth devices
    def scan(self):
        self._reset()
        self.need_scan = False
        print("Scanning for", self.target, "...")
        self.ble.gap_scan(5000, 30000, 30000, True)

    # calls interrupt for every event that happens during scan/connect
    def _irq(self, event, data):
        if event == _IRQ_SCAN_RESULT:
            addr_type, addr, adv_type, rssi, adv = data
            if self.addr is None and adv_name(adv) == self.target:
                self.addr_type = addr_type
                self.addr = bytes(addr)
                self.ble.gap_scan(None)

        elif event == _IRQ_SCAN_DONE:
            if self.addr is not None:
                print("Found car, connecting...")
                self.ble.gap_connect(self.addr_type, self.addr)
            else:
                self.need_scan = True

        elif event == _IRQ_PERIPHERAL_CONNECT:
            self.conn = data[0]
            self.ble.gattc_discover_services(self.conn)

        elif event == _IRQ_PERIPHERAL_DISCONNECT:
            print("Disconnected from car")
            self._reset()
            self.need_scan = True

        elif event == _IRQ_GATTC_SERVICE_RESULT:
            conn, start, end, uuid = data
            if uuid == _UART_UUID:
                self.svc_start, self.svc_end = start, end

        elif event == _IRQ_GATTC_SERVICE_DONE:
            if self.svc_start is not None:
                self.ble.gattc_discover_characteristics(self.conn, self.svc_start, self.svc_end)
            else:
                print("UART service not found")
                self.ble.gap_disconnect(self.conn)

        elif event == _IRQ_GATTC_CHARACTERISTIC_RESULT:
            conn, def_handle, value_handle, props, uuid = data
            if uuid == _RX_UUID:
                self.rx = value_handle
            elif uuid == _TX_UUID:
                self.tx = value_handle

        elif event == _IRQ_GATTC_CHARACTERISTIC_DONE:
            if self.rx is not None and self.tx is not None:
                # Turn on notifications: the CCCD sits right after the TX value handle
                self.ble.gattc_write(self.conn, self.tx + 1, struct.pack("<h", 1), 1)
                self.ready = True
                print("Connected and ready!")
            else:
                print("UART characteristics not found")
                self.ble.gap_disconnect(self.conn)

        elif event == _IRQ_GATTC_NOTIFY:
            conn, handle, msg = data
            if handle == self.tx:
                self.on_message(bytes(msg))

    def send(self, x, y, btn):
        if not self.ready:
            return
        try:
            self.ble.gattc_write(self.conn, self.rx, struct.pack("bbB", x, y, btn), 0)
        except OSError:
            pass

celebrate_pending = False

def on_message(msg):
    # runs inside the bluetooth interrupt: only set a flag, celebrate in the main loop
    global celebrate_pending
    print("From car:", msg)
    if msg == "FINISH":
        celebrate_pending = True


joy = Joystick()
ctrl = BLEController(TARGET_NAME, on_message)
ctrl.scan()

blink = 0
last_status = None
try:
    while True:
        if ctrl.need_scan:
            ctrl.scan()

        # car crossed the finish line
        if celebrate_pending:
            celebrate_pending = False
            celebrate()
            last_status = None          # redraw the status color afterwards

        x, y, btn = joy.read()
        ctrl.send(x, y, btn)

        # status pixels: dim green when connected, blinking blue while searching
        if ctrl.ready:
            status = CONNECTED
        else:
            blink = (blink + 1) % 20
            status = SEARCHING if blink < 10 else OFF
        if status != last_status:       # only rewrite the pixels when the color changes
            leds(status)
            last_status = status

        time.sleep_ms(SEND_PERIOD_MS)
finally:
    buzzer.duty_u16(0)
    leds(OFF)