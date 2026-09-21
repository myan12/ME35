import time
import wifi_setup
import urequests
from machine import Pin, PWM
from neopixel import NeoPixel

SERVO1_PIN = 4  # tens digit
SERVO2_PIN = 5  # ones digit
NEOPIXEL_PIN = 15
NUM_PIXELS = 2

BUTTON_PIN = 35
BUTTON_PRESSED_STATE = 1  # 1 = pin reads HIGH when pressed; set to 0 if wired the other way
DEBOUNCE_MS = 200

SERVO_MIN_US = 500
SERVO_MAX_US = 2500

BLUE = (0, 0, 30)
OFF = (0, 0, 0)

CITIES = [
    {"name": "Boston, MA",      "lat": 42.3601, "lon": -71.0589},
    {"name": "New York, NY",    "lat": 40.7128, "lon": -74.0060},
    {"name": "Chicago, IL",     "lat": 41.8781, "lon": -87.6298},
    {"name": "Seattle, WA",     "lat": 47.6062, "lon": -122.3321},
    {"name": "Miami, FL",       "lat": 25.7617, "lon": -80.1918},
    {"name": "Los Angeles, CA", "lat": 34.0522, "lon": -118.2437},
]

wifi_setup.connect_wifi()

servo1 = PWM(Pin(SERVO1_PIN), freq=50)
servo2 = PWM(Pin(SERVO2_PIN), freq=50)
np = NeoPixel(Pin(NEOPIXEL_PIN), NUM_PIXELS)

button = Pin(BUTTON_PIN, Pin.IN)

current_city_index = 0
pending_update = True  # fetch the first city's temp on startup
_last_press_ms = 0

def set_pixel(index, color):
    np[index] = color
    np.write()


def set_servo_angle(servo, angle, reversed=False):
    angle = max(0, min(180, angle))
    if reversed:
        angle = 180 - angle
    us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * angle // 180
    duty = int(us * 65535 // 20000)
    servo.duty_u16(duty)

def on_button_press(pin):
    """IRQ handler: just advances the city index and sets a flag.
    The actual network fetch happens in the main loop, not here."""
    global current_city_index, pending_update, _last_press_ms

    now = time.ticks_ms()
    if time.ticks_diff(now, _last_press_ms) < DEBOUNCE_MS:
        return
    if pin.value() != BUTTON_PRESSED_STATE:
        return
    _last_press_ms = now

    current_city_index = (current_city_index + 1) % len(CITIES)
    pending_update = True


button.irq(trigger=Pin.IRQ_RISING, handler=on_button_press)

def get_temp_for_city(city):
    temp_url = (
        "https://api.open-meteo.com/v1/forecast"
        "?latitude=" + str(city["lat"]) +
        "&longitude=" + str(city["lon"]) +
        "&current=temperature_2m&temperature_unit=fahrenheit"
    )
    reply = urequests.get(temp_url)
    data = reply.json()
    reply.close()
    return data['current']['temperature_2m']

def update_temperature_display(temp_f):
    temp_rounded = round(temp_f)
    temp_rounded = max(10, min(99, temp_rounded))

    tens_digit = (temp_rounded // 10) % 10
    ones_digit = temp_rounded % 10

    tens_angle = (tens_digit - 1) * 180 // 8
    ones_angle = ones_digit * 20

    print(
        "  -> rounded:", temp_rounded,
        "| tens digit:", tens_digit, "-> servo1 angle:", tens_angle,
        "| ones digit:", ones_digit, "-> servo2 angle:", ones_angle
    )

    set_servo_angle(servo1, tens_angle)
    set_servo_angle(servo2, ones_angle, reversed=True)  # servo2 is mirror-mounted

def refresh_current_city():
    city = CITIES[current_city_index]
    print("Selected city:", city["name"])
    set_pixel(0, BLUE)
    set_pixel(1, BLUE)

    try:
        temp_f = get_temp_for_city(city)
        print("Temperature in", city["name"], ":", temp_f)
        update_temperature_display(temp_f)
    except Exception as e:
        print("Error fetching temperature for", city["name"], ":", e)

def main():
    global pending_update

    print("City temperature display.")
    print("Press the D35 button to cycle to the next city.")
    print("Cities:", [c["name"] for c in CITIES])

    while True:
        if pending_update:
            pending_update = False
            try:
                refresh_current_city()
            except KeyboardInterrupt:
                print("Stopping.")
                return
            except Exception as e:
                print("Unexpected error:", e)

        time.sleep(0.05)


if __name__ == "__main__":
    main()