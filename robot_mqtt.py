# robot_mqtt.py - MicroPython on the ESP32 car
# drives the car from MQTT commands sent by a computer
# save to the board as main.py so it runs on power-up

import json
import time
import network
import machine
import ubinascii
from machine import Pin, PWM
from neopixel import NeoPixel
from umqtt.simple import MQTTClient
import secrets

# wifi network the laptop/broker is on (must be 2.4 GHz, WPA2-personal)
WIFI_SSID = secrets.SSID
WIFI_PASSWORD = secrets.pwd

# mqtt broker (laptop running mosquitto) - change this on demo day
BROKER_IP = "10.5.10.52"
BROKER_PORT = 1883

# command topics
TOPIC_SPEED = b"ME35/speed"
TOPIC_DIRECTION = b"ME35/direction"
TOPIC_LED0 = b"ME35/led/0"
TOPIC_LED1 = b"ME35/led/1"

# unique name for this board on the broker
CLIENT_ID = b"maggiebot-" + ubinascii.hexlify(machine.unique_id())
# the car pings itself on this topic to check the broker is still there
TOPIC_HEARTBEAT = b"maggiebot/" + CLIENT_ID + b"/heartbeat"

# motor pins
LEFT_MOTOR = (12, 13)
RIGHT_MOTOR = (14, 27)
# if a wheel spins backwards
LEFT_INVERT = False
RIGHT_INVERT = True
# speed multiplier per wheel to correct drift
LEFT_TRIM = 1.0
RIGHT_TRIM = 1.0
# speed used if no speed command has been sent yet
DEFAULT_SPEED = 20

# neopixels
NEOPIXEL_PIN = 15
NUM_PIXELS = 2
LOST_COLOR = (255, 0, 0)
BLINK_MS = 250

# connection timing
KEEPALIVE_S = 10          # broker drops the car if it goes silent this long
HEARTBEAT_MS = 1000       # how often the car pings itself
LOST_MS = 3000            # no heartbeat back in this long = connection lost
MQTT_RETRY_MS = 2000      # time between broker reconnect attempts
WIFI_RETRY_MS = 10000     # time between wifi reconnect attempts


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


# neopixels
pixels = NeoPixel(Pin(NEOPIXEL_PIN), NUM_PIXELS)
# colors set over mqtt, restored after a reconnect
led_colors = [(0, 0, 0)] * NUM_PIXELS


def show_leds(colors):
    for i in range(NUM_PIXELS):
        pixels[i] = colors[i]
    pixels.write()


# robot state
speed = DEFAULT_SPEED
direction = "stop"
last_heard = time.ticks_ms()


# runs the motors for the current direction and speed
def apply_motion():
    s = speed
    if direction == "forward":
        drive(s, s)
    elif direction == "backward":
        drive(-s, -s)
    elif direction == "left":        # rotate left in place
        drive(-s, s)
    elif direction == "right":       # rotate right in place
        drive(s, -s)
    else:
        stop()


# turns a JSON string like {"r": 255, "g": 0, "b": 122} into an (r, g, b) tuple
def parse_color(msg):
    data = json.loads(msg)
    return tuple(max(0, min(255, int(data.get(k, 0)))) for k in ("r", "g", "b"))


# called by umqtt for every message on a subscribed topic
def on_message(topic, msg):
    global speed, direction, last_heard
    last_heard = time.ticks_ms()
    if topic == TOPIC_HEARTBEAT:
        return
    try:
        text = msg.decode().strip()
        print(topic.decode(), text)

        # speed only changes the speed, it never starts the car
        if topic == TOPIC_SPEED:
            speed = max(0, min(100, int(float(text))))
            if direction != "stop":
                apply_motion()

        elif topic == TOPIC_DIRECTION:
            text = text.lower()
            if text in ("forward", "backward", "left", "right", "stop"):
                direction = text
                apply_motion()
            else:
                print("Unknown direction:", text)

        elif topic in (TOPIC_LED0, TOPIC_LED1):
            i = 0 if topic == TOPIC_LED0 else 1
            led_colors[i] = parse_color(text)
            show_leds(led_colors)

    # a bad message shouldn't crash the car
    except Exception as e:
        print("Bad message on", topic, msg, e)


# stops the car and closes the old connection
def connection_lost(client, reason):
    global direction
    print("Connection lost:", reason)
    direction = "stop"
    stop()
    if client is not None:
        try:
            client.sock.close()
        except Exception:
            pass


# one attempt at getting wifi + mqtt back; returns a client or None
def try_connect(wlan, now, state):
    if not wlan.isconnected():
        if time.ticks_diff(now, state["wifi_try"]) >= WIFI_RETRY_MS:
            state["wifi_try"] = now
            print("Connecting to wifi", WIFI_SSID, "...")
            try:
                wlan.connect(WIFI_SSID, WIFI_PASSWORD)
            except OSError as e:
                print("Wifi error:", e)
        return None

    if time.ticks_diff(now, state["mqtt_try"]) < MQTT_RETRY_MS:
        return None
    state["mqtt_try"] = now

    print("Wifi ok (", wlan.ifconfig()[0], "), connecting to broker", BROKER_IP, "...")
    try:
        client = MQTTClient(CLIENT_ID, BROKER_IP, port=BROKER_PORT, keepalive=KEEPALIVE_S)
        client.set_callback(on_message)
        client.connect()
        for t in (TOPIC_SPEED, TOPIC_DIRECTION, TOPIC_LED0, TOPIC_LED1, TOPIC_HEARTBEAT):
            client.subscribe(t)
        print("Connected to broker and subscribed")
        return client
    except Exception as e:
        print("Broker connect failed:", e)
        return None


# main
def run():
    global last_heard
    stop()
    show_leds(led_colors)

    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)

    client = None
    # start the retry timers in the past so the first attempt happens right away
    start = time.ticks_add(time.ticks_ms(), -WIFI_RETRY_MS)
    state = {"wifi_try": start, "mqtt_try": start}
    last_ping = time.ticks_ms()
    last_blink = time.ticks_ms()
    blink_on = False

    while True:
        now = time.ticks_ms()

        if client is not None:
            # connected: handle messages and check the link is still alive
            try:
                if not wlan.isconnected():
                    raise OSError("wifi dropped")
                client.check_msg()
                if time.ticks_diff(now, last_ping) >= HEARTBEAT_MS:
                    last_ping = now
                    client.publish(TOPIC_HEARTBEAT, b"1")
                if time.ticks_diff(now, last_heard) > LOST_MS:
                    raise OSError("broker stopped responding")
            except Exception as e:
                connection_lost(client, e)
                client = None

        else:
            # not connected: stay stopped and blink red
            stop()
            if time.ticks_diff(now, last_blink) >= BLINK_MS:
                last_blink = now
                blink_on = not blink_on
                show_leds([LOST_COLOR if blink_on else (0, 0, 0)] * NUM_PIXELS)

            client = try_connect(wlan, now, state)
            if client is not None:
                # back online: restore the lights, wait for a direction command
                last_heard = time.ticks_ms()
                last_ping = last_heard
                show_leds(led_colors)

        time.sleep_ms(10)


# turn everything off if the program stops
try:
    run()
finally:
    stop()
    show_leds([(0, 0, 0)] * NUM_PIXELS)