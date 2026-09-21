from machine import Pin, PWM
from neopixel import NeoPixel

# setting pins on the ESP32
HOUR_SERVO_PIN = 4
MINUTE_SERVO_PIN = 5
NEOPIXEL_PIN = 15
NUM_PIXELS = 2
SERVO_MIN_US = 500   # pulse width (us) at 0 degrees -- tune to your servo
SERVO_MAX_US = 2500  # pulse width (us) at 180 degrees -- tune to your servo

# colors
OFF = (0, 0, 0)
GREEN = (0, 30, 0)
RED = (30, 0, 0)
BLUE = (0, 0, 30)
PURPLE = (20, 0, 20)

hour_servo = PWM(Pin(HOUR_SERVO_PIN), freq=50)
minute_servo = PWM(Pin(MINUTE_SERVO_PIN), freq=50)
np = NeoPixel(Pin(NEOPIXEL_PIN), NUM_PIXELS)


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

    if hour12 <= 6:
        angle = hour12 * 30              # 0, 30, 60, 90, 120, 150, 180 (12 -> 6, forward sweep)
        set_pixel(0, GREEN)
    else:
        angle = 180 - (hour12 - 6) * 30  # 150, 120, 90, 60, 30 (6 -> 12, backward sweep)
        set_pixel(0, RED)

    print("  hour_servo -> angle:", angle)
    set_servo_angle(hour_servo, angle)


def update_minute_hand(minute):
    if minute <= 30:
        angle = 180 - minute * 6          # 180, 174, ..., 0 (0 -> 30, mirrored)
        set_pixel(1, GREEN)
    else:
        angle = (minute - 30) * 6         # 6, ..., 180 (30 -> 60, mirrored)
        set_pixel(1, RED)

    print("  minute_servo -> angle:", angle)
    set_servo_angle(minute_servo, angle)


def update_clock_from_string(time_str):
    """time_str expected as 'HH:MM' (24-hour), e.g. '14:35'"""
    hour_str, minute_str = time_str.split(':')
    hour = int(hour_str)
    minute = int(minute_str)

    print("Testing time:", time_str, "-> hour:", hour, "minute:", minute)

    update_hour_hand(hour)
    update_minute_hand(minute)


def main():
    print("Servo/NeoPixel test harness.")
    print("Enter a time as HH:MM (24-hour), e.g. 14:35")
    print("Enter 'q' to quit.\n")

    while True:
        user_input = input("Time (HH:MM) > ").strip()

        if user_input.lower() == 'q':
            print("Exiting test harness.")
            break

        try:
            update_clock_from_string(user_input)
        except Exception as e:
            print("Couldn't parse/apply that time:", e)
            print("Make sure to use the format HH:MM, e.g. 09:05 or 21:47\n")


if __name__ == "__main__":
    main()