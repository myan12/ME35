import wifi_setup
import urequests
import time as time_module
from machine import Pin, PWM
from neopixel import NeoPixel

# time pins
SERVO1_PIN = 4  # hour hand (time mode) / tens digit (temperature mode)
SERVO2_PIN = 5  # minute hand (time mode) / ones digit (temperature mode)
NEOPIXEL_PIN = 15
NUM_PIXELS = 2

MODE_BUTTON_PIN = 34 # modes: time/temperature
CITY_BUTTON_PIN = 35 # temperature in different cities
BUTTON_PRESSED_STATE = 1 
DEBOUNCE_MS = 200

SERVO_MIN_US = 500   # pulse width (us) at 0 degrees
SERVO_MAX_US = 2500  # pulse width (us) at 180 degrees
POLL_INTERVAL_SEC = 60  # how often to re-check the active mode's data source
DATE_URL = "https://timeapi.io/api/time/current/zone?timeZone=America/New_York"

# colors
OFF = (0, 0, 0)
GREEN = (0, 30, 0)
RED = (30, 0, 0)
BLUE = (0, 0, 30)

MODE_CLOCK = "clock"
MODE_TEMPERATURE = "temperature"

# dictionary of cities with corresponding latitude/longitude
# for api lookup of temperature in different cities
CITIES = [
    {"name": "Boston, MA",      "lat": 42.3601, "lon": -71.0589},
    {"name": "New York, NY",    "lat": 40.7128, "lon": -74.0060},
    {"name": "Chicago, IL",     "lat": 41.8781, "lon": -87.6298},
    {"name": "Seattle, WA",     "lat": 47.6062, "lon": -122.3321},
    {"name": "Miami, FL",       "lat": 25.7617, "lon": -80.1918},
    {"name": "Los Angeles, CA", "lat": 34.0522, "lon": -118.2437},
]

# initialization of pins on ESP32
servo1 = PWM(Pin(SERVO1_PIN), freq=50)
servo2 = PWM(Pin(SERVO2_PIN), freq=50)
np = NeoPixel(Pin(NEOPIXEL_PIN), NUM_PIXELS)

mode_button = Pin(MODE_BUTTON_PIN, Pin.IN, Pin.PULL_UP)
city_button = Pin(CITY_BUTTON_PIN, Pin.IN, Pin.PULL_UP)

# initializing global variables 
current_mode = MODE_CLOCK
current_city_index = 0
pending_update = True
_last_mode_press_ms = 0
_last_city_press_ms = 0

# connect to wifi
wifi_setup.connect_wifi()

#---------------------------------------------------
# handler functions
#------------------------------------- -----------------

# use API to fetch current time from URL
def get_current_time():
    reply = urequests.get(DATE_URL)
    data = reply.json()
    reply.close()
    return data['time']  # e.g. "14:23:07"

# use API to fetch current temperature of requested city from corresponding URL
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

# set pixel colors
def set_pixel(index, color):
    np[index] = color
    np.write()

# sets the servo angle by converting angle into microseconds (pulse width)
# and then converts into a duty cycle that micropython can understand
#
# reversed=True flips the physical direction for a servo mounted/wired opposite
def set_servo_angle(servo, angle, reversed=False):
    angle = max(0, min(180, angle))
    if reversed:
        angle = 180 - angle
    us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * angle // 180
    duty = int(us * 65535 // 20000)  # 20000us = one 50Hz period
    servo.duty_u16(duty)

# interrupt handler for D34 mode pin (toggles between time/temp)
# no data fetch/network calls in handler - only toggles global variable
def on_mode_button_press(pin):
    global current_mode, pending_update, _last_mode_press_ms

    now = time_module.ticks_ms()
    if time_module.ticks_diff(now, _last_mode_press_ms) < DEBOUNCE_MS:
        return
    if pin.value() != BUTTON_PRESSED_STATE:
        return
    _last_mode_press_ms = now

    current_mode = MODE_TEMPERATURE if current_mode == MODE_CLOCK else MODE_CLOCK
    pending_update = True
    print("Button pressed -> switched to", current_mode, "mode")

# interrupt handler for D35 city pin (toggles between list of cities)
# no data fetch/network calls in handler - only toggles global variable
# pin only active in temperature mode
def on_city_button_press(pin):
    global current_city_index, pending_update, _last_city_press_ms

    now = time_module.ticks_ms()
    if time_module.ticks_diff(now, _last_city_press_ms) < DEBOUNCE_MS:
        return
    if pin.value() != BUTTON_PRESSED_STATE:
        return
    _last_city_press_ms = now

    # ignore button press if not in temperature mode
    if current_mode != MODE_TEMPERATURE:
        return
    
    # cycle through list of cities
    current_city_index = (current_city_index + 1) % len(CITIES)
    pending_update = True

# interrupt calls for each button
mode_button.irq(trigger=Pin.IRQ_RISING, handler=on_mode_button_press)
city_button.irq(trigger=Pin.IRQ_RISING, handler=on_city_button_press)

# ---------------------------------------------------------------------------
# functions for clock mode
# ---------------------------------------------------------------------------

# drives servo 1 based on input hour
def update_hour_hand(hour_24):
    hour12 = hour_24 % 12

    # sets neopixel 0 corresponding to inner/outer circle of clock
    if hour12 <= 6:
        # forward sweep
        # 12:00 = 0 degrees, 6:00 = 180 degrees
        angle = hour12 * 30
        set_pixel(0, GREEN)
    else:
        # sweeps in reverse from 6:00 - 12:00
        angle = 180 - (hour12 - 6) * 30
        set_pixel(0, RED)
    
    print("  servo1 -> angle:", angle)
    set_servo_angle(servo1, angle)

# drives servo 2 based on input minute
def update_minute_hand(minute):
    # sets neopixel 1 corresponding to inner/outer circle of clock
    # mirrored angles from hour servo
    if minute <= 30:
        # forward sweep from minute 0 to minute 30
        angle = 180 - minute * 6
        set_pixel(1, GREEN)
    else:
        # backward sweep from minute 30 to minute 60 
        angle = (minute - 30) * 6
        set_pixel(1, RED)

    print("  servo2 -> angle:", angle)
    set_servo_angle(servo2, angle)

# pipeline to fetch time from API and update clock hands to match
def update_clock():
    # splits time into hour and minute integers
    current_time = get_current_time()
    print(current_time)
    hour_str, minute_str = current_time.split(':')
    hour = int(hour_str)
    minute = int(minute_str)

    print("Current time:", current_time, "-> hour:", hour, "minute:", minute)

    # calls functions to update servos
    update_hour_hand(hour)
    update_minute_hand(minute)

# ---------------------------------------------------------------------------
# temperature mode
# ---------------------------------------------------------------------------

# updates the temperature based on raw temperature reading from API
# displays as two digits (one per servo)
def update_temperature_display(temp_f):
    # rounds temperature to nearest integer value
    temp_rounded = round(temp_f)
    temp_rounded = max(10, min(99, temp_rounded))  # clamp to 10-99

    # ranges from 1-9 for tens digit across 0-180 degrees
    tens_digit = (temp_rounded // 10) % 10
    # ranges from 0-9 for ones digit across 0-180 degrees
    ones_digit = temp_rounded % 10

    # corresponding servo angles for digit values
    tens_angle = (tens_digit - 1) * 180 // 8
    ones_angle = ones_digit * 20

    print(
        "Temperature:", temp_f, "-> rounded:", temp_rounded,
        "| tens digit:", tens_digit, "-> servo1 angle:", tens_angle,
        "| ones digit:", ones_digit, "-> servo2 angle:", ones_angle
    )

    set_servo_angle(servo1, tens_angle)
    set_servo_angle(servo2, ones_angle, reversed=True)  # servo2 is mirror-mounted

# pipeline to update the temperature display based on city
def update_temperature():
    city = CITIES[current_city_index]
    print("Selected city:", city["name"])
    set_pixel(0, BLUE)
    set_pixel(1, BLUE)
    
    # fetch data from API based on city
    temp_f = get_temp_for_city(city)
    print("Temperature in", city["name"], ":", temp_f)
    update_temperature_display(temp_f)

# main loop - runs indefinitely
def main():
    global pending_update

    # track last displayed information to detect changes
    last_mode = None
    last_city_index = None
    last_refresh_time = 0

    while True:
        now = time_module.time()
        # check if mode changed or if the city changed since last refresh
        mode_or_city_changed = (
            current_mode != last_mode or
            (current_mode == MODE_TEMPERATURE and current_city_index != last_city_index)
        )
        # check if 60 seconds has elapsed since last update
        due_for_refresh = (now - last_refresh_time) >= POLL_INTERVAL_SEC

        # variables that can trigger a refresh: button interrupt fired, city/mode changed (fallback if
        # button press was missed), or refresh time (60s) has elapsed
        if pending_update or mode_or_city_changed or due_for_refresh:
            pending_update = False # reset flag for next button press
            try:
                # update clock or temperature based on current mode
                if current_mode == MODE_CLOCK:
                    update_clock()
                else:
                    update_temperature()
            except KeyboardInterrupt:
                # stops program instead of looping forever
                print("Stopping clock loop.")
                return
            except Exception as e:
                print("Error updating display:", e)

            # updates recent displayed values
            last_mode = current_mode
            last_city_index = current_city_index
            last_refresh_time = now

        time_module.sleep(0.05)

if __name__ == "__main__":
    mai