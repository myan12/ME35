import wifi_setup
import urequests
import time as time_module
from machine import Pin, PWM
from neopixel import NeoPixel

# setting pins on the ESP32
HOUR_SERVO_PIN = 4
MINUTE_SERVO_PIN = 5
NEOPIXEL_PIN = 15
NUM_PIXELS = 2
SERVO_MIN_US = 500   # pulse width (us) at 0 degrees -- tune to your servo
SERVO_MAX_US = 2500  # pulse width (us) at 180 degrees -- tune to your servo
POLL_INTERVAL_SEC = 60  # how often to re-check the time and update hands
DATE_URL = "https://timeapi.io/api/time/current/zone?timeZone=America/New_York"
# colors
OFF = (0,0,0)
GREEN = (0, 30, 0)
RED = (30, 0, 0)
BLUE = (0, 0, 30)
PURPLE = (20, 0, 20)

hour_servo = PWM(Pin(HOUR_SERVO_PIN), freq=50)
minute_servo = PWM(Pin(MINUTE_SERVO_PIN), freq=50)
np = NeoPixel(Pin(NEOPIXEL_PIN), NUM_PIXELS)

wifi_setup.connect_wifi()

def get_current_time():
    reply = urequests.get(DATE_URL)
    data = reply.json()
    reply.close()
    return data['time']  # e.g. "14:23:07"

# set pixel colors
def set_pixel(index, color):
    np[index] = color
    np.write()
    
# sets the servo angle by converting angle into microseconds (pulse width)
# and then converts into a duty cycle that micropython can understand
def set_servo_angle(servo, angle):
    angle = max(0, min(180, angle))
    us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * angle // 180
    duty = int(us * 65535 // 20000)  # 20000us = one 50Hz period
    servo.duty_u16(duty)
    
def update_hour_hand(hour_24):
    hour12 = hour_24 % 12  # 0-11 (0 = 12 o'clock)
 
    if hour12 < 6:
        angle = hour12 * 30           # 0, 30, 60, 90, 120, 150
        set_pixel(0, GREEN)
    else:
        angle = (hour12 - 6) * 30     # 0, 30, 60, 90, 120, 150
        set_pixel(0, RED)
 
    set_servo_angle(hour_servo, angle)
    
def update_minute_hand(minute):
    if minute < 30:
        angle = minute * 6            # 0, 6, 12, ..., 174
        set_pixel(1, GREEN)
    else:
        angle = (minute - 30) * 6     # 0, 6, 12, ..., 174
        set_pixel(1, RED)
 
    set_servo_angle(minute_servo, angle)

def update_clock():
    current_time = get_current_time()
    hour_str, minute_str, _ = current_time.split(':')
    hour = int(hour_str)
    minute = int(minute_str)
 
    print("Current time:", current_time, "-> hour:", hour, "minute:", minute)
 
    update_hour_hand(hour)
    update_minute_hand(minute)
    
def main():
    while True:
        try:
            update_clock()
        except Exception as e:
            print("Error updating clock:", e)
 
        time.sleep(POLL_INTERVAL_SEC)
 
 
if __name__ == "__main__":
    main()
