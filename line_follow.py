import time
import network
import espnow
from machine import Pin, PWM, I2C
from neopixel import NeoPixel

# ======================= SETTINGS =======================
TEAM = "T1"                   # must match TEAM in station.py
DEBUG = True                  # print sensor values (use this to tune thresholds)

# Sensors: VEML6040 has a fixed I2C address (0x10), so each gets its own bus.
LEFT_SDA, LEFT_SCL = 21, 22   # I2C bus 0
RIGHT_SDA, RIGHT_SCL = 26, 25 # I2C bus 1 = Grove port 3 (verify with an I2C scan)

# Motors (swap INVERT if a wheel spins backwards)
LEFT_MOTOR = (12, 13)         # M1A, M1B
RIGHT_MOTOR = (14, 27)        # M2A, M2B
LEFT_TRIM = 1.0               # speed multiplier per wheel to correct a drift (e.g. 0.95)
RIGHT_TRIM = 1.0
LEFT_INVERT = False
RIGHT_INVERT = True

BUZZER_PIN = 23
NEOPIXEL_PIN = 15
BTN_CAL = Pin(34, Pin.IN)
BTN_GO = Pin(35, Pin.IN)
PRESSED = 0                   # buttons read 0 when pressed (flip to 1 if yours are opposite)

SERVO_PIN = 4                 # flag servo on D4
FLAG_REST = 0                 # angle when the flag is down (TUNE)
FLAG_LOW, FLAG_HIGH = 45, 135 # the wave swings between these two angles
FLAG_WAVES = 4                # how many back-and-forth waves
FLAG_STEP_MS = 250            # time per swing (lower = faster waving)

# Station (second ESP32) - ESP-NOW
STATION_MAC = b'\x20\x50\x0d\x29\x93\x8c'
ACK_TRIES = 3                 # how many times to send before giving up
ACK_WAIT_MS = 300             # how long to wait for the ACK after each send

# driving settings (speeds are percent, 0-100)
BASE_SPEED = 25
KP = 55.0                     # steering strength
KD = 45.0                     # damping (reduces wobble)
SEARCH_SPEED = 40
FRONT_SENSOR = "left"         # which sensor is mounted further forward ("left", "right", or None if level)
FRONT_DELAY_LOOPS = 4         # loops to delay the front sensor (~25 ms each). TUNE: more if corners still fail
# sweep used for when robot gets lost, up to 4 sweeps, where the angle gets wider for each turn
SWEEP_MS = 400                # when the line is lost: the first sweep lasts this long (TUNE ~ a 90 degree pivot)
SWEEP_MAX_STEPS = 4           # each sweep is longer than the last (1x, 2x, 3x... SWEEP_MS), up to this many x.
                              # Bigger = wider search. 4 reaches about 2 x SWEEP_MS's angle on each side.
SWEEP_FIRST = 1               # which way the search sweeps first: 1 = left, -1 = right
LOST_THRESH = 0.15            # both sensors lighter than this = line lost

# for white/black: brightness (W) only
# saturation threshold, max color distance and dark cutoff come from training_data.csv
# always match the training
KNN_K = 5                     # number of nearest samples that vote
KNN_EVERY = 1                 # run KNN every N loops (use 2 if the loop feels slow)
# color accepted with 3 votes; a stray reading (patch border, glare) is simply outvoted
CONFIRM_SPEED = 12            # slows down when color detected for more samples
CONFIRM_WINDOW_MS = 400       # how long to look before giving up
CONFIRM_VOTES = 3             # votes needed (the first sighting counts as one). Raise if false detections return
COOLDOWN_MS = 2500            # ignore colors after successfully detected
SPIN_360_MS = 900             # time to spin one full turn (TUNE THIS)


class VEML6040:
    ADDR = 0x10

    # initializes color sensor to i2c bus
    def __init__(self, i2c):
        self.i2c = i2c
        # Config register 0x00: SD=0 (on), auto mode, 40 ms integration
        self.i2c.writeto_mem(self.ADDR, 0x00, b"\x00\x00")

    def read(self):
        """Returns [R, G, B, W] raw 16-bit values."""
        out = []
        for reg in (0x08, 0x09, 0x0A, 0x0B):
            d = self.i2c.readfrom_mem(self.ADDR, reg, 2)
            out.append(d[0] | (d[1] << 8))
        return out


class Motor:
    def __init__(self, pin_a, pin_b, invert=False):
        self.a = PWM(Pin(pin_a), freq=10000, duty_u16=0)
        self.b = PWM(Pin(pin_b), freq=10000, duty_u16=0)
        self.invert = invert

    def set(self, speed):
        """speed: -100..100 percent"""
        speed = max(-100, min(100, speed))
        if self.invert:
            speed = -speed
        duty = int(abs(speed) * 65535 / 100)
        if speed > 0:
            self.a.duty_u16(duty); self.b.duty_u16(0)
        elif speed < 0:
            self.a.duty_u16(0); self.b.duty_u16(duty)
        else:
            self.a.duty_u16(0); self.b.duty_u16(0)   # brake


left_motor = Motor(*LEFT_MOTOR, invert=LEFT_INVERT)
right_motor = Motor(*RIGHT_MOTOR, invert=RIGHT_INVERT)


def drive(l, r):
    left_motor.set(l * LEFT_TRIM)
    right_motor.set(r * RIGHT_TRIM)


def stop():
    drive(0, 0)


# buzzer and neopixels
buzzer = PWM(Pin(BUZZER_PIN), freq=1000, duty_u16=0)
pixels = NeoPixel(Pin(NEOPIXEL_PIN), 2)


def tone(freq, ms):
    buzzer.freq(freq)
    buzzer.duty_u16(32768)
    time.sleep_ms(ms)
    buzzer.duty_u16(0)


def leds(r, g, b):
    pixels[0] = (r, g, b)
    pixels[1] = (r, g, b)
    pixels.write()

# servo setup
servo = PWM(Pin(SERVO_PIN), freq=50, duty_u16=0)


def servo_angle(deg):
    """Move the servo to 0-180 degrees (0.5-2.5 ms pulse at 50 Hz)."""
    deg = max(0, min(180, deg))
    us = 500 + deg * 2000 / 180
    servo.duty_u16(int(us * 65535 / 20000))


def servo_rest():
    servo_angle(FLAG_REST)
    time.sleep_ms(400)
    servo.duty_u16(0)         # stop the pulses so the servo doesn't buzz or jitter


servo_rest()                  # flag down at startup

# connect to wifi (ESP-NOW, talks directly to the station)
sta = network.WLAN(network.STA_IF)
sta.active(True)
sta.disconnect()              # ESP-NOW only, no Wi-Fi network
en = espnow.ESPNow()
en.active(True)
# for connecting to peers
en.add_peer(STATION_MAC)
print("Robot MAC:", ":".join("%02x" % b for b in sta.config("mac")),
      " -> station", ":".join("%02x" % b for b in STATION_MAC))
seq = 0


def ack_flash():
    """Quick white flash + chirp so you can SEE the robot got the station's ACK."""
    leds(40, 40, 40)
    tone(2000, 60)
    leds(0, 0, 0)


def report(color_name):
    """Send the detected color to the station and wait for its ACK. Returns True if acked.
    Message:  'T1,RED,7'   Expected reply:  'ACK,T1,7'  (same sequence number)."""
    global seq
    seq += 1
    msg = "%s,%s,%d" % (TEAM, color_name, seq)
    want = "ACK,%s,%d" % (TEAM, seq)

    while en.any():           # throw away any old/late messages so they can't be mistaken for this ACK
        en.recv(0)

    for attempt in range(1, ACK_TRIES + 1):
        try:
            delivered = en.send(STATION_MAC, msg)   # True = station's radio received the packet
            print("Sent", msg, "(try %d, radio delivered: %s)" % (attempt, delivered))
        except OSError as e:
            print("send error", e)
        t0 = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), t0) < ACK_WAIT_MS:
            mac, data = en.recv(50)
            if mac == STATION_MAC and data and data.decode() == want:
                print("ACK received:", want)
                ack_flash()
                return True
    print("No ACK for", msg)
    return False


# color sensing
i2c_l = I2C(0, sda=Pin(LEFT_SDA), scl=Pin(LEFT_SCL), freq=400000)
i2c_r = I2C(1, sda=Pin(RIGHT_SDA), scl=Pin(RIGHT_SCL), freq=400000)
sens_l = VEML6040(i2c_l)
sens_r = VEML6040(i2c_r)

white_l = [1, 1, 1, 1]
white_r = [1, 1, 1, 1]


# normalize colors based on env lighting
def normalize(raw, white):
    return [raw[i] / white[i] for i in range(4)]


def calibrate_white():
    global white_l, white_r
    sl = [0, 0, 0, 0]
    sr = [0, 0, 0, 0]
    n = 0
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < 2000:
        a = sens_l.read()
        b = sens_r.read()
        for i in range(4):
            sl[i] += a[i]
            sr[i] += b[i]
        n += 1
        time.sleep_ms(45)
    white_l = [max(v / n, 1) for v in sl]
    white_r = [max(v / n, 1) for v in sr]
    print("White L:", white_l, " R:", white_r)


# ---------- color detection (same features as collect_samples.py) ----------
COLOR_NAMES = ("RED", "GREEN", "BLUE")
# data is pulled from CSV file, trained by collect_samples.py
DATA_FILE = "training_data.csv"


MIN_BRIGHT = 0.05             # dark cutoff; overwritten from training_data.csv


def chroma(r, g, b):
    """Hue only: r, g, b as fractions of their sum (brightness removed). None if too dark."""
    s = r + g + b
    if s < MIN_BRIGHT:
        return None
    return (r / s, g / s, b / s)


def saturation(c):
    """0 = gray (white, black, gray edges); larger = more colorful."""
    if c is None:
        return 0.0
    return max(c) - min(c)


SAT_THRESH = 0.12             # overwritten from training_data.csv
COLOR_MAX_DIST = 0.10         # overwritten from training_data.csv
COLOR_SET = []                # (label, cr, cg, cb): RED/GREEN/BLUE + colorful EDGE/TURN samples
NOT_COLOR_EXAMPLES = ("EDGE", "TURN")   # kept only if colorful, so they outvote false colors


def load_training():
    global SAT_THRESH, COLOR_MAX_DIST, MIN_BRIGHT
    try:
        with open(DATA_FILE) as f:
            for line in f:
                p = line.strip().split(",")
                if p[0] == "#settings" and len(p) >= 3:
                    SAT_THRESH = float(p[1])
                    COLOR_MAX_DIST = float(p[2])
                    if len(p) >= 4:
                        MIN_BRIGHT = float(p[3])
                elif len(p) == 5 and (p[0] in COLOR_NAMES or p[0] in NOT_COLOR_EXAMPLES):
                    c = chroma(float(p[1]), float(p[2]), float(p[3]))
                    if c is not None and (p[0] in COLOR_NAMES or saturation(c) > SAT_THRESH):
                        COLOR_SET.append((p[0], c[0], c[1], c[2]))
        print("Loaded %d color samples, SAT_THRESH=%.3f, COLOR_MAX_DIST=%.3f, MIN_BRIGHT=%.3f"
              % (len(COLOR_SET), SAT_THRESH, COLOR_MAX_DIST, MIN_BRIGHT))
    except OSError:
        print("No %s found - run collect_samples.py first!" % DATA_FILE)


load_training()


def color_label(n):
    """Returns RED / GREEN / BLUE, or None (white, black, edge, or not a trained color).
    Step 1: not colorful enough -> None (white/black are handled by brightness in darkness()).
    Step 2: k-nearest-neighbors on hue only (r, g, b fractions)."""
    c = chroma(n[0], n[1], n[2])
    if saturation(c) <= SAT_THRESH or not COLOR_SET:
        return None
    best = []                             # the K nearest (dist^2, label), kept sorted
    for t in COLOR_SET:
        d = (t[1] - c[0]) ** 2 + (t[2] - c[1]) ** 2 + (t[3] - c[2]) ** 2
        if len(best) < KNN_K or d < best[-1][0]:
            best.append((d, t[0]))
            best.sort()
            if len(best) > KNN_K:
                best.pop()
    if best[0][0] > COLOR_MAX_DIST ** 2:
        return None
    votes = {}
    for d, lbl in best:
        votes[lbl] = votes.get(lbl, 0) + 1
    win = best[0][1]                      # ties go to the single nearest sample
    for lbl in votes:
        if votes[lbl] > votes[win]:
            win = lbl
    return win if win in COLOR_NAMES else None


def confirm_color(first):
    """Crawl slowly over the patch, taking a fresh reading from both sensors every 45 ms.
    Returns the color once it has CONFIRM_VOTES votes, or None if nothing wins in time."""
    drive(CONFIRM_SPEED, CONFIRM_SPEED)
    votes = {first: 1}
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < CONFIRM_WINDOW_MS:
        time.sleep_ms(45)                 # the sensor makes a new reading every 40 ms
        for lbl in (color_label(normalize(sens_l.read(), white_l)),
                    color_label(normalize(sens_r.read(), white_r))):
            if lbl:
                votes[lbl] = votes.get(lbl, 0) + 1
                if votes[lbl] >= CONFIRM_VOTES:
                    if DEBUG:
                        print("confirm votes:", votes)
                    return lbl
    if DEBUG:
        print("not confirmed, votes:", votes)
    return None


def darkness(nrm):
    """0 = white paper, 1 = black tape (uses the clear/white channel)."""
    return max(0.0, min(1.0, 1.0 - nrm[3]))


# color detection manuevers
def maneuver_red():
    """Stop and wave the flag with red lights."""
    stop()
    leds(60, 0, 0)
    for _ in range(FLAG_WAVES):
        servo_angle(FLAG_HIGH)
        time.sleep_ms(FLAG_STEP_MS)
        servo_angle(FLAG_LOW)
        time.sleep_ms(FLAG_STEP_MS)
    servo_rest()
    leds(0, 0, 0)


def maneuver_green():
    """Wiggle while flashing green."""
    for _ in range(4):
        leds(0, 60, 0)
        drive(55, -55)
        time.sleep_ms(150)
        leds(0, 0, 0)
        drive(-55, 55)
        time.sleep_ms(150)
    stop()


def maneuver_blue():
    """Play a short rising tune with blue lights."""
    leds(0, 0, 60)
    for f in (523, 659, 784, 1047, 784, 1047):
        tone(f, 140)
        time.sleep_ms(30)
    leds(0, 0, 0)


MANEUVERS = {"RED": maneuver_red, "GREEN": maneuver_green, "BLUE": maneuver_blue}


# ---------- main ----------
def wait_button(btn, msg):
    print(msg)
    leds(20, 20, 0)
    while btn.value() != PRESSED:
        time.sleep_ms(20)
    while btn.value() == PRESSED:   # wait for release
        time.sleep_ms(20)
    leds(0, 0, 0)


def run():
    wait_button(BTN_CAL, "Place BOTH sensors on white, press D34 to calibrate")
    calibrate_white()
    tone(1500, 120)
    wait_button(BTN_GO, "Place robot on the line, press D35 to start")
    time.sleep_ms(500)

    last_err = 0.0
    cooldown_until = 0
    dbg_t = 0
    delay_buf = None           # remembers the front sensor's recent darkness values
    delay_i = 0
    lost_t0 = None             # when the line was lost (for the sweep search)
    lbl_l = lbl_r = None       # latest color labels (for debug printing)
    last_color = None
    knn_tick = 0

    while True:
        nl = normalize(sens_l.read(), white_l)
        nr = normalize(sens_r.read(), white_r)

        # --- 1. color detection: hue-only KNN on either sensor ---
        color = None
        if time.ticks_diff(time.ticks_ms(), cooldown_until) > 0:
            knn_tick += 1
            if knn_tick >= KNN_EVERY:
                knn_tick = 0
                lbl_l = color_label(nl)
                lbl_r = color_label(nr)
                if lbl_l in COLOR_NAMES:
                    last_color = lbl_l
                elif lbl_r in COLOR_NAMES:
                    last_color = lbl_r
                else:
                    last_color = None
            color = last_color
        else:
            last_color = None

        if color:
            # a color was seen: slow to a crawl and take a vote over several fresh readings
            name = confirm_color(color)
            if name:
                stop()
                print("Detected", name)
                report(name)              # notify the station and wait for its ACK BEFORE the maneuver
                MANEUVERS[name]()
                drive(BASE_SPEED, BASE_SPEED)   # roll off the patch
                time.sleep_ms(300)
                cooldown_until = time.ticks_add(time.ticks_ms(), COOLDOWN_MS)
            last_color = None
            continue

        # --- 2. line following (brightness only) ---
        dl = darkness(nl)
        dr = darkness(nr)

        # The front sensor sees the line earlier than the rear one. Delay its reading
        # so both describe the same spot on the line (fixes wrong turns at sharp corners).
        if FRONT_SENSOR and FRONT_DELAY_LOOPS > 0:
            front = dl if FRONT_SENSOR == "left" else dr
            if delay_buf is None:
                delay_buf = [front] * FRONT_DELAY_LOOPS
            old = delay_buf[delay_i]
            delay_buf[delay_i] = front
            delay_i = (delay_i + 1) % FRONT_DELAY_LOOPS
            if FRONT_SENSOR == "left":
                dl = old
            else:
                dr = old

        if DEBUG and time.ticks_diff(time.ticks_ms(), dbg_t) > 250:
            dbg_t = time.ticks_ms()
            print("L rgbw", ["%.2f" % v for v in nl], "R rgbw", ["%.2f" % v for v in nr],
                  "dL %.2f dR %.2f" % (dl, dr),
                  "| sat L %.3f R %.3f" % (saturation(chroma(nl[0], nl[1], nl[2])),
                                           saturation(chroma(nr[0], nr[1], nr[2]))),
                  "| color L:", lbl_l, "R:", lbl_r)

        if dl < LOST_THRESH and dr < LOST_THRESH:
            # lost the line: expanding sweep. Turn SWEEP_FIRST's way for 1 x SWEEP_MS, then the
            # other way for 2x, then back for 3x, 4x... Each sweep crosses the starting heading and
            # reaches further out, so it checks near the line first, then wider.
            # After SWEEP_MAX_STEPS it keeps sweeping at that widest width.
            now = time.ticks_ms()
            if lost_t0 is None:
                lost_t0 = now
            t = time.ticks_diff(now, lost_t0)
            k = 1                    # which sweep we are in (1, 2, 3...)
            while k < SWEEP_MAX_STEPS and t >= k * SWEEP_MS:
                t -= k * SWEEP_MS
                k += 1
            if k == SWEEP_MAX_STEPS:
                k += t // (SWEEP_MAX_STEPS * SWEEP_MS)   # keep alternating at the widest width
            d = SWEEP_FIRST if k % 2 == 1 else -SWEEP_FIRST
            drive(-d * SEARCH_SPEED, d * SEARCH_SPEED)
        else:
            lost_t0 = None
            err = dl - dr            # >0: line toward left sensor -> steer left
            steer = KP * err + KD * (err - last_err)
            last_err = err
            drive(BASE_SPEED - steer, BASE_SPEED + steer)

        time.sleep_ms(20)


try:
    run()
finally:
    stop()
    buzzer.duty_u16(0)
    servo.duty_u16(0)
    leds(0, 0, 0)