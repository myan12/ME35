# Station module: second ESP32 that receives color reports from the line-following robot
# and acknowledges each one over ESP-NOW.
# MicroPython (v1.20 or newer). Save to the station board as main.py in Thonny
# (or run it as station.py while it's plugged into your laptop).
#
# Robot sends:    "T1,RED,7"     (team, color, sequence number)
# Station replies: "ACK,T1,7"    (same sequence number) straight back to the robot
#
# The acknowledgement is shown three ways on the station:
#   1. NeoPixels light up in the detected color
#   2. a short beep
#   3. a console printout with a running tally of detections
#
# You do NOT need to type in the robot's MAC address: the station replies to whichever
# board sent the message (only messages tagged with this TEAM are accepted).

import time
import network
import espnow
from machine import Pin, PWM

# ======================= SETTINGS =======================
TEAM = "T1"                   # must match TEAM in the robot's main.py
BUZZER_PIN = 23               # ROBO ESP32 pins; change if the station is a different board
NEOPIXEL_PIN = 15
NUM_PIXELS = 2
SHOW_MS = 1500                # how long the color stays lit after a report
DUPLICATE_MS = 2000           # a repeated sequence number within this time = robot's retry
COLOR_RGB = {"RED": (60, 0, 0), "GREEN": (0, 60, 0), "BLUE": (0, 0, 60)}
# =========================================================

# ---------- ESP-NOW ----------
sta = network.WLAN(network.STA_IF)
sta.active(True)
sta.disconnect()              # ESP-NOW only, no Wi-Fi network
en = espnow.ESPNow()
en.active(True)


def fmt_mac(mac):
    return ":".join("%02x" % b for b in mac)


print("Station MAC:", fmt_mac(sta.config("mac")), "(should be 20:50:0d:29:93:8c)")

# ---------- LEDs + buzzer (skipped if the board doesn't have them) ----------
try:
    from neopixel import NeoPixel
    pixels = NeoPixel(Pin(NEOPIXEL_PIN), NUM_PIXELS)
except Exception as e:
    pixels = None
    print("No NeoPixels:", e)

try:
    buzzer = PWM(Pin(BUZZER_PIN), freq=1000, duty_u16=0)
except Exception as e:
    buzzer = None
    print("No buzzer:", e)


def leds(rgb):
    if pixels:
        for i in range(NUM_PIXELS):
            pixels[i] = rgb
        pixels.write()


def beep(freq=1800, ms=80):
    if buzzer:
        buzzer.freq(freq)
        buzzer.duty_u16(32768)
        time.sleep_ms(ms)
        buzzer.duty_u16(0)


# ---------- helpers ----------
known_peers = set()


def ensure_peer(mac):
    """ESP-NOW can only send to registered peers, so register the robot the first time it talks."""
    if mac in known_peers:
        return
    try:
        en.add_peer(mac)
    except OSError:
        pass                  # already registered
    known_peers.add(mac)
    print("Robot connected:", fmt_mac(mac))


# ---------- main loop ----------
counts = {"RED": 0, "GREEN": 0, "BLUE": 0}
last_seq = None
last_seq_t = 0
leds_off_at = None

leds((10, 10, 10))            # dim white = station ready
time.sleep_ms(300)
leds((0, 0, 0))
print("Station ready (team %s), waiting for the robot..." % TEAM)

try:
    while True:
        mac, data = en.recv(100)          # wait up to 100 ms for a message

        if leds_off_at is not None and time.ticks_diff(time.ticks_ms(), leds_off_at) >= 0:
            leds((0, 0, 0))
            leds_off_at = None

        if not data:
            continue

        try:
            parts = data.decode().split(",")
        except Exception:
            continue
        if len(parts) != 3 or parts[0] != TEAM or parts[1] not in COLOR_RGB:
            print("Ignored:", data)       # another team's message or garbage
            continue
        color, seq = parts[1], parts[2]

        # 1. acknowledge FIRST so the robot isn't kept waiting
        ensure_peer(mac)
        try:
            en.send(mac, "ACK,%s,%s" % (TEAM, seq))
        except OSError as e:
            print("ACK send error", e)

        # 2. the robot resends if our ACK got lost; ACK it again but don't count it twice
        now = time.ticks_ms()
        if seq == last_seq and time.ticks_diff(now, last_seq_t) < DUPLICATE_MS:
            print("Repeat of #%s (re-sent ACK)" % seq)
            continue
        last_seq, last_seq_t = seq, now

        # 3. show the acknowledgement
        counts[color] += 1
        print("#%s  %-5s received -> ACK sent   | totals R:%d G:%d B:%d"
              % (seq, color, counts["RED"], counts["GREEN"], counts["BLUE"]))
        leds(COLOR_RGB[color])
        beep()
        leds_off_at = time.ticks_add(time.ticks_ms(), SHOW_MS)
finally:
    leds((0, 0, 0))
    if buzzer:
        buzzer.duty_u16(0)