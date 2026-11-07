from machine import Pin, PWM, I2C, ADC
import neopixel
import time
import math

# pins
I2C_SCL = 22
I2C_SDA = 21
NEOPIXEL_PIN = 15
NUM_PIXELS = 2            
BUZZER_PIN = 23           
SERVO_PIN = 4

USE_JOYSTICK = True
JOY_X_PIN = 36          
JOY_Y_PIN = 39
BUTTON_PINS = (34, 35)    

# joystick (your measured values)
# calibrates joystick using these values to determine what counts as a "press" vs. a "hold"
JOY_PRESS_LEVEL = 0.97   
JOY_LOW = 0.21
JOY_HIGH = 0.79
JOY_DEADZONE = 0.35
JOY_SETTLE_MS = 40       
                          
JOY_REPEAT_SLOW_MS = 400
JOY_REPEAT_FAST_MS = 120
HOLD_MS = 1000          

# accelerometer
UP_AXIS = 2              
UP_SIGN = 1
# if the apple was more sensitive, could use tap
TAP_THS = 40              # hardware tap threshold, 16 mg/count at +/-2 g (40 = 0.64 g)
TAP_TIME_LIMIT = 10       # max length of a tap spike, 10 ms ticks at 100 Hz
# tap vs. double tap (not used for this prototype)
TAP_WINDOW_MS = 450       
SHAKE_THRESH_G = 0.8
SHAKE_PEAKS = 3
SHAKE_WINDOW_MS = 700
SHAKE_COOLDOWN_MS = 1500
TIP_COS = 0.6      
UPRIGHT_COS = 0.9
# determines tip and hold vs. a quick tip
TIP_HOLD_MS = 1200
# ignoring taps for this version
USE_TAPS = False       

# differentiates tap from other movements that can jolt the shell (ignore this)
TAP_QUIET_MS = 500  

# servo dial
DIAL_ZERO_ANGLE = 0
DIAL_FULL_ANGLE = 180
DIAL_WIGGLE = 15
MAX_MIN = 60              
NOD_DEG = 10             
NOD_MS = 150           

# cooking presets
PRESETS = (
    # name,       minutes, color shown while choosing it
    ("soft egg",  6.5,     (200, 140, 0)),
    ("hard egg",  10,      (220, 50, 0)),
    ("pasta",     10,      (170, 170, 50)),
    ("rice",      18,      (140, 140, 140)),
    ("tea",       4,       (0, 110, 90)),
    ("custom",    5,       (120, 0, 160)),  
)
CUSTOM = len(PRESETS) - 1
SNOOZE_MIN = 1            # "not done yet" adds this much
OVERCOOK_MIN = 2          # ignored this long after the alarm = fully burnt (brown)

# in demo mode, mins become sec for easy demonstration
DEMO_MODE = True
# debug mode prints all values of sensors for debugging
DEBUG = False

def unit_ms(minutes):
    return int(minutes * (1000 if DEMO_MODE else 60000))


def lerp(c1, c2, t):
    t = max(0.0, min(1.0, t))
    return tuple(int(a + (b - a) * t) for a, b in zip(c1, c2))


def scale(c, k):
    return tuple(int(v * k) for v in c)


GREEN = (0, 160, 0)
RED = (200, 0, 0)
BROWN = (90, 35, 5)
OFF = (0, 0, 0)


# accelerometer class using i2c communication protocol
class Accel:
    WHO_AM_I = 0x0F
    CTRL_REG1 = 0x20
    CTRL_REG2 = 0x21
    CTRL_REG4 = 0x23
    OUT_X_L = 0x28
    CLICK_CFG = 0x38
    CLICK_SRC = 0x39
    CLICK_THS = 0x3A
    TIME_LIMIT = 0x3B
    TIME_LATENCY = 0x3C
    AUTO_INC = 0x80

    def __init__(self, i2c):
        self.i2c = i2c
        found = i2c.scan()
        print("I2C devices:", [hex(a) for a in found])
        self.addr = None
        for addr in (0x19, 0x18):
            if addr in found:
                self.addr = addr
                break
        if self.addr is None:
            raise OSError("LIS3DH not found at 0x18/0x19 - check wiring")
        who = i2c.readfrom_mem(self.addr, self.WHO_AM_I, 1)[0]
        if who != 0x33:
            print("Warning: WHO_AM_I = {} (expected 0x33)".format(hex(who)))
        w = self._write
        w(self.CTRL_REG1, 0x57)              # 100 Hz, X/Y/Z on
        w(self.CTRL_REG4, 0x88)              # BDU, +/-2 g, high-res
        # Tap detection runs on the chip. The high-pass filter feeds the click
        # engine so gravity (a constant 1 g) never looks like a tap.
        w(self.CTRL_REG2, 0x04)              # HPCLICK
        w(self.CLICK_CFG, 0x15)              # single click on X, Y and Z
        w(self.CLICK_THS, 0x80 | TAP_THS)    # bit 7: latch until CLICK_SRC is read
        w(self.TIME_LIMIT, TAP_TIME_LIMIT)
        w(self.TIME_LATENCY, 5)              # 50 ms dead time after each tap
        time.sleep_ms(10)
        self.tapped()                        # clear anything latched at startup
        print("LIS3DH ready at", hex(self.addr))

    def _write(self, reg, val):
        self.i2c.writeto_mem(self.addr, reg, bytes([val]))

    @staticmethod
    def _s16(v):
        return v - 65536 if v > 32767 else v

    def read(self):
        d = self.i2c.readfrom_mem(self.addr, self.OUT_X_L | self.AUTO_INC, 6)
        raw = [self._s16(d[i] | (d[i + 1] << 8)) for i in (0, 2, 4)]
        return tuple(r / 16384 for r in raw)

    def tapped(self):
        """True if a tap happened since the last call (reading clears the latch)."""
        src = self.i2c.readfrom_mem(self.addr, self.CLICK_SRC, 1)[0]
        return bool(src & 0x40) and bool(src & 0x10)   # IA and SCLICK


# accelerometer gestures that include tip, shake and taps
class Gestures:
    def __init__(self, accel):
        self.accel = accel
        self.peaks = []
        self.above = False
        self.last_shake = time.ticks_add(time.ticks_ms(), -10 * SHAKE_COOLDOWN_MS)
        self.taps = []
        self.smooth_up = 1.0
        self.tipped_at = None
        self.holding = False
        self.last_tip_end = self.last_shake

    def update(self, now):
        events = []
        try:
            a = self.accel.read()
            tapped = self.accel.tapped() if USE_TAPS else False
        except OSError:
            return events

        # shake: several big swings away from 1 g in a short window
        mag = math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2)
        dyn = abs(mag - 1.0)
        if dyn > SHAKE_THRESH_G and not self.above:
            self.peaks.append(now)
            self.above = True
        elif dyn < SHAKE_THRESH_G * 0.5:
            self.above = False
        self.peaks = [t for t in self.peaks if time.ticks_diff(now, t) < SHAKE_WINDOW_MS]
        if (len(self.peaks) >= SHAKE_PEAKS
                and time.ticks_diff(now, self.last_shake) > SHAKE_COOLDOWN_MS):
            events.append("shake")
            self.last_shake = now
            self.peaks = []
            self.taps = []          # a shake's knocks are not taps
        shaking = time.ticks_diff(now, self.last_shake) < SHAKE_COOLDOWN_MS

        # taps: group the chip's clicks, decide once the window closes.
        # clicks while tilting or just after setting it down don't count.
        settling = (self.tipped_at is not None
                    or time.ticks_diff(now, self.last_tip_end) < TAP_QUIET_MS)
        if settling:
            self.taps = []
        if tapped and not shaking and not settling:
            self.taps.append(now)
        if self.taps and time.ticks_diff(now, self.taps[0]) > TAP_WINDOW_MS:
            n = len(self.taps)
            self.taps = []
            if not shaking:
                if n == 1:
                    events.append("tap")
                elif n == 2:
                    events.append("double_tap")
                # 3+ clicks = jostling, ignore

        # Tipping back. Two gestures from one motion:
        #   quick tip (back and upright again within TIP_HOLD_MS) -> "tip"
        #   tip and hold (still tilted after TIP_HOLD_MS)         -> "tilt_start",
        #   then "tilt_end" when it's set back upright
        up = UP_SIGN * a[UP_AXIS]
        self.smooth_up = 0.8 * self.smooth_up + 0.2 * up
        if self.tipped_at is None:
            if self.smooth_up < TIP_COS:
                self.tipped_at = now
                self.holding = False
        elif self.smooth_up > UPRIGHT_COS:
            shook = time.ticks_diff(self.last_shake, self.tipped_at) > -SHAKE_COOLDOWN_MS
            if self.holding:
                events.append("tilt_end")
            elif not shook:
                events.append("tip")
            self.tipped_at = None
            self.holding = False
            self.last_tip_end = now
        elif (not self.holding
              and time.ticks_diff(now, self.tipped_at) > TIP_HOLD_MS):
            self.holding = True
            events.append("tilt_start")
        return events


# joystick class
class Joystick:
    def __init__(self):
        self.x = ADC(Pin(JOY_X_PIN))
        self.y = ADC(Pin(JOY_Y_PIN))
        for adc in (self.x, self.y):
            adc.atten(ADC.ATTN_11DB)
        self.buttons = [Pin(p, Pin.IN) for p in BUTTON_PINS]
        self.pressed = False
        self.held = False
        self.press_start = 0
        self.last_edge = 0
        self.push_since = None
        self.repeating = False
        self.last_dir_time = 0
        self.last_active = time.ticks_add(time.ticks_ms(), -10000)
        self.last_debug = 0
        n = 20   # calibrate resting center - don't touch the stick at startup
        self.cx = sum(self.x.read_u16() for _ in range(n)) / n / 65535
        self.cy = sum(self.y.read_u16() for _ in range(n)) / n / 65535
        print("[joy] center calibrated: x={:.2f} y={:.2f}".format(self.cx, self.cy))

    def update(self, now):
        events = []
        x_raw = self.x.read_u16() / 65535
        y_raw = self.y.read_u16() / 65535

        if DEBUG and time.ticks_diff(now, self.last_debug) > 250:
            self.last_debug = now
            print("[joy] raw x={:.2f} y={:.2f} | push x={:+.2f} y={:+.2f}  pressed={}".format(
                x_raw, y_raw, self.norm(x_raw, self.cx), self.norm(y_raw, self.cy),
                self.pressed))

        # Press = X slammed to full scale, or an onboard button pulled low
        down = x_raw > JOY_PRESS_LEVEL or any(b.value() == 0 for b in self.buttons)
        if down or self.pressed:
            self.last_active = now
        if down != self.pressed and time.ticks_diff(now, self.last_edge) > 80:
            self.last_edge = now
            self.pressed = down
            self.push_since = None      # ignore the X swing around the edge
            if down:
                self.press_start = now
                self.held = False
            elif not self.held:
                events.append("press")
        if self.pressed:
            if not self.held and time.ticks_diff(now, self.press_start) > HOLD_MS:
                self.held = True
                events.append("hold")
            return events

        x = self.norm(x_raw, self.cx)
        y = self.norm(y_raw, self.cy)
        push = max(abs(x), abs(y))
        if push > JOY_DEADZONE:
            self.last_active = now
            if self.push_since is None:
                self.push_since = now
                self.repeating = False
            if time.ticks_diff(now, self.push_since) < JOY_SETTLE_MS:
                return events
            t = (push - JOY_DEADZONE) / (1 - JOY_DEADZONE)
            repeat_ms = JOY_REPEAT_SLOW_MS - t * (JOY_REPEAT_SLOW_MS - JOY_REPEAT_FAST_MS)
            if not self.repeating or time.ticks_diff(now, self.last_dir_time) > repeat_ms:
                if abs(y) >= abs(x):
                    d = "up" if y > 0 else "down"
                else:
                    d = "right" if x > 0 else "left"
                events.append(d)
                self.last_dir_time = now
                self.repeating = True
        else:
            self.push_since = None
        return events

    @staticmethod
    def norm(v, center):
        if v >= center:
            out = (v - center) / max(0.01, JOY_HIGH - center)
        else:
            out = (v - center) / max(0.01, center - JOY_LOW)
        return max(-1.0, min(1.0, out))


# buzzer for different tones / sounds to indicate different things
class Voice:
    def __init__(self, pin):
        self.pwm = PWM(Pin(pin), freq=1000, duty_u16=0)
        self.notes = []
        self.note_end = 0

    def play(self, notes):
        self.notes = list(notes)
        self.note_end = 0

    def busy(self):
        return bool(self.notes) or self.note_end != 0

    def update(self, now):
        if self.note_end and time.ticks_diff(now, self.note_end) < 0:
            return
        if not self.notes:
            self.pwm.duty_u16(0)
            self.note_end = 0
            return
        f, ms = self.notes.pop(0)
        if f:
            self.pwm.freq(f)
            self.pwm.duty_u16(20000)
        else:
            self.pwm.duty_u16(0)
        self.note_end = time.ticks_add(now, ms)


HELLO = [(880, 80), (0, 30), (1175, 120)]
START = [(784, 80), (988, 80), (1319, 160)]
CLICK = [(1500, 25)]
TADA = [(784, 120), (988, 120), (1175, 120), (1568, 300)]
ALARM = [(1319, 120), (0, 60), (1319, 120), (0, 400)]
GRUMPY_1 = [(600, 150), (450, 250)]
GRUMPY_2 = [(500, 200), (380, 200), (300, 400)]
GRUMPY_3 = [(400, 250), (0, 80), (300, 250), (0, 80), (200, 700)]
SULK = [(500, 300), (350, 500)]
WAKE = [(600, 80), (900, 120)]


def chirp(hz):
    return [(hz, 40)]


# servo
class Dial:
    def __init__(self, pin):
        self.pwm = PWM(Pin(pin), freq=50)
        self.angle = None
        self.goto(DIAL_ZERO_ANGLE)

    def goto(self, angle):
        angle = max(0, min(180, angle))
        if self.angle is not None and abs(angle - self.angle) < 1:
            return   # avoid jitter
        self.angle = angle
        pulse_ms = 0.5 + (angle / 180) * 2.0
        self.pwm.duty_u16(int(pulse_ms / 20 * 65535))
        if DEBUG:
            print("[servo] angle={:.0f}".format(angle))

    def show_minutes(self, minutes):
        frac = max(0.0, min(1.0, minutes / MAX_MIN))
        self.goto(DIAL_ZERO_ANGLE + frac * (DIAL_FULL_ANGLE - DIAL_ZERO_ANGLE))


# neopixel lights (ignore this because not visible from outside of apple)
class Lights:
    def __init__(self, pin, n):
        self.np = neopixel.NeoPixel(Pin(pin), n)
        self.n = n

    def fill(self, c):
        for i in range(self.n):
            self.np[i] = c
        self.np.write()

# timer class
class CookTimer:
    def __init__(self, name, minutes, now):
        self.name = name
        self.duration = unit_ms(minutes)
        self.start = now
        self.paused_at = None
        self.ringing = False
        self.ring_start = 0
        self.snoozes = 0

    def elapsed(self, now):
        end = self.paused_at if self.paused_at is not None else now
        return time.ticks_diff(end, self.start)

    def remaining(self, now):
        return max(0, self.duration - self.elapsed(now))

    def remaining_min(self, now):
        return self.remaining(now) / unit_ms(1)

    def progress(self, now):
        return min(1.0, self.elapsed(now) / self.duration) if self.duration else 1.0

    def add(self, ms, now):
        left = self.remaining(now) + ms
        left = max(unit_ms(0.5), min(unit_ms(MAX_MIN), left))
        self.duration = self.elapsed(now) + left

    def not_done_yet(self, now):
        self.snoozes += 1
        self.ringing = False
        self.start = now
        self.duration = unit_ms(SNOOZE_MIN)

    def pause(self, now):
        if self.paused_at is None:
            self.paused_at = now

    def resume(self, now):
        if self.paused_at is not None:
            self.start = time.ticks_add(self.start, time.ticks_diff(now, self.paused_at))
            self.paused_at = None

    def color(self, now):
        if self.ringing:
            ignored = time.ticks_diff(now, self.ring_start) / unit_ms(OVERCOOK_MIN)
            return lerp(RED, BROWN, max(ignored, self.snoozes / 3))
        base = RED if self.snoozes else lerp(GREEN, RED, self.progress(now))
        return lerp(base, BROWN, self.snoozes / 3)

SETUP, RUN, ALARM_ST = "SETUP", "RUN", "ALARM"


class Pomme:
    def __init__(self):
        i2c = I2C(0, scl=Pin(I2C_SCL), sda=Pin(I2C_SDA), freq=400000)
        self.gestures = Gestures(Accel(i2c))
        self.joy = Joystick() if USE_JOYSTICK else None
        self.voice = Voice(BUZZER_PIN)
        self.dial = Dial(SERVO_PIN)
        self.lights = Lights(NEOPIXEL_PIN, NUM_PIXELS)

        self.timer = None
        self.state = SETUP
        self.preset = 0
        self.setup_min = PRESETS[0][1]
        self.custom_min = PRESETS[CUSTOM][1]
        self.paused = False
        self.tilt_paused = False     # paused by tip-and-hold (vs. a press)
        self.flash = False
        self.last_flash = 0
        self.nod_start = 0
        self.nod_count = 0

        self.voice.play(HELLO)
        print("Hi! I'm Pomme. What are we cooking?")
        self.say_preset()

    # ---------- helpers ----------
    def say_preset(self):
        print("Preset: {} - {} min".format(PRESETS[self.preset][0], self.setup_min))

    def nod(self, count):
        """Twitch the dial `count` times as a little acknowledgement."""
        self.nod_start = time.ticks_ms()
        self.nod_count = count

    def nod_offset(self, now, base_angle):
        phase = time.ticks_diff(now, self.nod_start) // NOD_MS
        if phase >= 2 * self.nod_count or phase % 2:
            return 0
        return -NOD_DEG if base_angle > 90 else NOD_DEG

    def pause(self, now, by_tilt=False):
        if self.paused:
            return
        self.paused = True
        self.tilt_paused = by_tilt
        self.timer.pause(now)
        self.voice.play(SULK)
        print("Paused - off the heat.")

    def resume(self, now):
        if not self.paused:
            return
        self.paused = False
        self.tilt_paused = False
        self.timer.resume(now)
        self.voice.play(WAKE)
        print("Back on the heat!")

    def announce(self, now):
        """Beep out the time left: one long beep per 5 min, one short per minute."""
        m = math.ceil(self.timer.remaining_min(now))
        fives, ones = divmod(m, 5)
        self.voice.play([(660, 300), (0, 150)] * fives + [(990, 90), (0, 140)] * ones)
        self.nod(1)
        print("{} min left".format(m))

    def finish(self):
        self.timer = None
        self.state = SETUP
        self.paused = False
        self.tilt_paused = False
        self.say_preset()

    # ---------- events ----------
    def handle(self, ev, now):
        s = self.state

        if s == SETUP:
            if ev in ("left", "right"):
                self.preset = (self.preset + (1 if ev == "right" else -1)) % len(PRESETS)
                if self.preset == CUSTOM:
                    self.setup_min = self.custom_min
                else:
                    self.setup_min = PRESETS[self.preset][1]
                self.voice.play(chirp(700 + 150 * self.preset))
                self.say_preset()
            elif ev in ("up", "down"):
                small = self.setup_min < 5 or (ev == "down" and self.setup_min <= 5)
                step = 0.5 if small else 1
                self.setup_min += step if ev == "up" else -step
                self.setup_min = max(0.5, min(MAX_MIN, self.setup_min))
                if self.preset == CUSTOM:
                    self.custom_min = self.setup_min
                self.voice.play(CLICK)
                self.say_preset()
            elif ev == "press":
                self.timer = CookTimer(PRESETS[self.preset][0], self.setup_min, now)
                self.state = RUN
                self.voice.play(START)
                print("Cooking {} for {} min".format(self.timer.name, self.setup_min))
            return

        t = self.timer

        if s == ALARM_ST:
            if ev in ("press", "hold", "shake"):
                self.voice.play(TADA)
                print("{} is done - enjoy!".format(t.name))
                self.finish()
            elif ev in ("tip", "double_tap"):
                t.not_done_yet(now)
                self.state = RUN
                self.voice.play((GRUMPY_1, GRUMPY_2, GRUMPY_3)[min(t.snoozes, 3) - 1])
                print("{} not done yet (+{} min, {}x). Pomme smells burning...".format(
                    t.name, SNOOZE_MIN, t.snoozes))
            return

        # RUN
        if ev == "tilt_start":
            self.pause(now, by_tilt=True)
        elif ev == "tilt_end":
            if self.tilt_paused:
                self.resume(now)
        elif ev == "press":
            if self.paused:
                self.resume(now)
            else:
                self.pause(now)
        elif ev == "hold":
            self.voice.play(SULK)
            print("Cancelled {}.".format(t.name))
            self.finish()
        elif ev == "tap":
            self.announce(now)
        elif ev in ("up", "down", "double_tap", "tip"):
            t.add(unit_ms(-1 if ev == "down" else 1), now)
            self.voice.play(CLICK if ev in ("up", "down") else CLICK + [(0, 40)] + CLICK)
            print("{}: {:.1f} min left".format(t.name, t.remaining_min(now)))

    # ---------- every loop ----------
    def tick(self, now):
        if time.ticks_diff(now, self.last_flash) > 300:
            self.last_flash = now
            self.flash = not self.flash
        t = self.timer

        if self.state == RUN and not self.paused and t.remaining(now) == 0:
            t.ringing = True
            t.ring_start = now
            self.state = ALARM_ST
            print("Ding! {} is ready.".format(t.name))

        # Lights (hidden inside the apple, but they mirror the timer)
        if self.state == SETUP:
            breathe = 0.25 + 0.75 * (0.5 - 0.5 * math.cos(2 * math.pi * (now % 3000) / 3000))
            c = scale(PRESETS[self.preset][2], breathe)
        elif self.state == ALARM_ST:
            c = t.color(now) if self.flash else OFF
        elif self.paused:
            c = scale(t.color(now), 0.08)
        else:
            c = t.color(now)
        self.lights.fill(c)

        # Dial
        if self.state == ALARM_ST:
            self.dial.goto(DIAL_ZERO_ANGLE + (DIAL_WIGGLE if self.flash else 0))
        elif self.state == SETUP:
            self.dial.show_minutes(self.setup_min)
        else:
            m = t.remaining_min(now)
            base = DIAL_ZERO_ANGLE + min(1.0, m / MAX_MIN) * (DIAL_FULL_ANGLE - DIAL_ZERO_ANGLE)
            self.dial.goto(base + self.nod_offset(now, base))

        # Alarm voice: grumpier the longer it's ignored or snoozed
        if self.state == ALARM_ST and not self.voice.busy():
            ignored = time.ticks_diff(now, t.ring_start) > unit_ms(OVERCOOK_MIN) / 2
            if t.snoozes >= 2 or ignored:
                self.voice.play(GRUMPY_1 + [(0, 300)] + ALARM)
            else:
                self.voice.play(ALARM)

    def run(self):
        while True:
            now = time.ticks_ms()
            events = self.gestures.update(now)
            if self.joy:
                events += self.joy.update(now)
                # Working the stem jolts the shell; don't read that as a tap
                if time.ticks_diff(now, self.joy.last_active) < TAP_QUIET_MS:
                    events = [e for e in events if e not in ("tap", "double_tap")]
            for ev in events:
                if DEBUG:
                    print("[event] {} (state={}, paused={})".format(ev, self.state, self.paused))
                self.handle(ev, now)
            self.tick(now)
            self.voice.update(now)
            time.sleep_ms(20)


# calibration of different gestures and accelerometer for debugging
def print_accel(seconds=10):
    """>>> import main; main.print_accel()   (find UP_AXIS / UP_SIGN)"""
    acc = Accel(I2C(0, scl=Pin(I2C_SCL), sda=Pin(I2C_SDA), freq=400000))
    end = time.ticks_add(time.ticks_ms(), seconds * 1000)
    while time.ticks_diff(end, time.ticks_ms()) > 0:
        x, y, z = acc.read()
        print("x={:+.2f} y={:+.2f} z={:+.2f} |a|={:.2f}".format(
            x, y, z, math.sqrt(x * x + y * y + z * z)))
        time.sleep_ms(200)


def print_gestures(seconds=20):
    """>>> import main; main.print_gestures()   (tune TAP_THS, tip, shake)
    Tap the apple, double-tap, tip it toward you, shake it."""
    g = Gestures(Accel(I2C(0, scl=Pin(I2C_SCL), sda=Pin(I2C_SDA), freq=400000)))
    end = time.ticks_add(time.ticks_ms(), seconds * 1000)
    while time.ticks_diff(end, time.ticks_ms()) > 0:
        for ev in g.update(time.ticks_ms()):
            print(ev)
        time.sleep_ms(20)


def print_joystick(seconds=10):
    """>>> import main; main.print_joystick()"""
    x, y = ADC(Pin(JOY_X_PIN)), ADC(Pin(JOY_Y_PIN))
    x.atten(ADC.ATTN_11DB)
    y.atten(ADC.ATTN_11DB)
    end = time.ticks_add(time.ticks_ms(), seconds * 1000)
    while time.ticks_diff(end, time.ticks_ms()) > 0:
        print("x={:.2f} y={:.2f}".format(x.read_u16() / 65535, y.read_u16() / 65535))
        time.sleep_ms(200)


def test_servo():
    """>>> import main; main.test_servo()"""
    d = Dial(SERVO_PIN)
    for a in (0, 45, 90, 135, 180, 90, 0):
        d.goto(a)
        time.sleep_ms(700)


if __name__ == "__main__":
    Pomme().run()