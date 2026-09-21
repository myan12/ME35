import time
from machine import Pin, PWM
from neopixel import NeoPixel

# setting pins on the ESP32
SERVO1_PIN = 4  # tens digit
SERVO2_PIN = 5  # ones digit
NEOPIXEL_PIN = 15
NUM_PIXELS = 2

BUTTON_PIN = 34
BUTTON_PRESSED_STATE = 1  # 1 = pin reads HIGH when pressed; set to 0 if wired the other way
DEBOUNCE_MS = 200

SERVO_MIN_US = 500   # pulse width (us) at 0 degrees -- tune to your servo
SERVO_MAX_US = 2500  # pulse width (us) at 180 degrees -- tune to your servo

BLUE = (0, 0, 30)
OFF = (0, 0, 0)

servo1 = PWM(Pin(SERVO1_PIN), freq=50)
servo2 = PWM(Pin(SERVO2_PIN), freq=50)
np = NeoPixel(Pin(NEOPIXEL_PIN), NUM_PIXELS)

button = Pin(BUTTON_PIN, Pin.IN, Pin.PULL_UP)

temperature_mode_active = False
_last_press_ms = 0


def set_pixel(index, color):
    np[index] = color
    np.write()


def on_button_press(pin):
    """IRQ handler: toggles temperature mode and updates the NeoPixels."""
    global temperature_mode_active, _last_press_ms

    now = time.ticks_ms()
    if time.ticks_diff(now, _last_press_ms) < DEBOUNCE_MS:
        return  # debounce: ignore presses that come in too fast
    if pin.value() != BUTTON_PRESSED_STATE:
        return  # ignore the release edge if IRQ fires on both edges
    _last_press_ms = now

    temperature_mode_active = not temperature_mode_active

    if temperature_mode_active:
        set_pixel(0, BLUE)
        set_pixel(1, BLUE)
        print("\n[Button] Temperature mode ON -- both LEDs blue")
    else:
        set_pixel(0, OFF)
        set_pixel(1, OFF)
        print("\n[Button] Temperature mode OFF -- LEDs off")


button.irq(trigger=Pin.IRQ_RISING, handler=on_button_press)


# sets the servo angle by converting angle into microseconds (pulse width)
# and then converts into a duty cycle that micropython can understand
#
# reversed=True flips the physical direction for a servo mounted/wired
# opposite to the others (same idea as in the clock test harness).
def set_servo_angle(servo, angle, reversed=False):
    angle = max(0, min(180, angle))
    if reversed:
        angle = 180 - angle
    us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * angle // 180
    duty = int(us * 65535 // 20000)  # 20000us = one 50Hz period
    servo.duty_u16(duty)


def update_temperature_display(temp_f):
    """
    Rounds temp_f to the nearest integer and splits it into tens/ones
    digits, each shown on its own servo across the full 0-180 range.

    tens digit 1-9 -> servo1 angle 0-180 (represents 10/20/.../90; 0 deg = 10)
    ones digit 0-9 -> servo2 angle 0-180 (represents 0-9)
    """
    temp_rounded = round(temp_f)
    temp_rounded = max(10, min(99, temp_rounded))  # clamp to 10-99 so tens digit is 1-9

    tens_digit = (temp_rounded // 10) % 10
    ones_digit = temp_rounded % 10

    # tens_digit ranges 1-9 (values 10-90), spread across 0-180 in 8 equal
    # steps so tens_digit=1 (10) -> 0 deg and tens_digit=9 (90) -> 180 deg
    tens_angle = (tens_digit - 1) * 180 // 8
    ones_angle = ones_digit * 20   # 0, 20, 40, ..., 180

    print(
        "Temperature:", temp_f, "-> rounded:", temp_rounded,
        "| tens digit:", tens_digit, "-> servo1 angle:", tens_angle,
        "| ones digit:", ones_digit, "-> servo2 angle:", ones_angle
    )

    set_servo_angle(servo1, tens_angle)
    set_servo_angle(servo2, ones_angle, reversed=True)


def main():
    print("Temperature servo test harness.")
    print("Press the D34 button first to turn temperature mode ON (LEDs go blue).")
    print("Then enter a temperature (e.g. 72 or 68.4).")
    print("Enter 'q' to quit.\n")

    while True:
        user_input = input("Temperature > ").strip()

        if user_input.lower() == 'q':
            print("Exiting test harness.")
            break

        if not temperature_mode_active:
            print("Temperature mode is OFF -- press the D34 button first.\n")
            continue

        try:
            temp_f = float(user_input)
            update_temperature_display(temp_f)
        except Exception as e:
            print("Couldn't parse/apply that value:", e)
            print("Make sure to enter a plain number, e.g. 72 or 68.4\n")


if __name__ == "__main__":
    main()