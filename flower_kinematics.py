from machine import Pin, PWM
import math
import time

SERVO1_PIN = 4          
SERVO2_PIN = 5          
LED_PIN = 27 

MIN_PULSE_US = 500      
MAX_PULSE_US = 2500    

L1 = 91.3                
L2 = 40.0                
GEAR_RATIO = 2.0

THETA1_OFFSET = 180      # deg, link 1 angle when servo 1 = 0 (180 = pointing left)
SERVO1_DIR = 1           # servo 1 increasing swings link 1 left -> down -> right (as observed)
THETA2_OFFSET = 0        # deg, link 2 angle RELATIVE to link 1 when servo 2 = 0 (0 = in line)
SERVO2_DIR = -1          # as observed: servo2=45 turns link 2 90 deg to the left (link 1 down)
ELBOW_SIGN = 1           # elbow branch; must match arm_workspace.py
SERVO1_RANGE = (0, 180)
SERVO2_RANGE = (0, 180)

STEP_MM = 1.0            # max straight-line distance per small step
STEP_DELAY_MS = 20       # pause per step (bigger = slower, smoother)
SETTLE_MS = 300          # extra wait after a travel move before the LED turns on
START_DELAY_S = 2        # time to get ready (e.g. open camera shutter)

POINTS = [
    (79.00, 5.00, 0),
    (75.90, 4.22, 1),
    (73.06, 1.95, 1),
    (70.68, -1.59, 1),
    (68.93, -6.08, 1),
    (67.89, -11.14, 1),
    (67.54, -16.34, 1),
    (64.13, -15.23, 1),
    (60.61, -15.16, 1),
    (57.22, -16.26, 1),
    (54.24, -18.56, 1),
    (51.92, -21.92, 1),
    (50.47, -26.09, 1),
    (50.01, -30.76, 1),
    (50.56, -35.52, 1),
    (52.08, -40.00, 1),
    (54.39, -43.88, 1),
    (57.27, -46.93, 1),
    (60.45, -49.04, 1),
    (58.70, -53.56, 1),
    (57.57, -58.56, 1),
    (57.22, -63.74, 1),
    (57.75, -68.70, 1),
    (59.17, -73.04, 1),
    (61.37, -76.41, 1),
    (64.18, -78.51, 1),
    (67.37, -79.18, 1),
    (70.68, -78.41, 1),
    (73.85, -76.31, 1),
    (76.68, -73.14, 1),
    (79.00, -69.25, 1),
    (81.32, -73.14, 1),
    (84.15, -76.31, 1),
    (87.32, -78.41, 1),
    (90.63, -79.18, 1),
    (93.82, -78.51, 1),
    (96.63, -76.41, 1),
    (98.83, -73.04, 1),
    (100.25, -68.70, 1),
    (100.78, -63.74, 1),
    (100.43, -58.56, 1),
    (99.30, -53.56, 1),
    (97.55, -49.04, 1),
    (100.73, -46.93, 1),
    (103.61, -43.88, 1),
    (105.92, -40.00, 1),
    (107.44, -35.52, 1),
    (107.99, -30.76, 1),
    (107.53, -26.09, 1),
    (106.08, -21.92, 1),
    (103.76, -18.56, 1),
    (100.78, -16.26, 1),
    (97.39, -15.16, 1),
    (93.87, -15.23, 1),
    (90.46, -16.34, 1),
    (90.11, -11.14, 1),
    (89.07, -6.08, 1),
    (87.32, -1.59, 1),
    (84.94, 1.95, 1),
    (82.10, 4.22, 1),
    (79.00, 5.00, 1),
    (79.00, -29.50, 0),
    (74.98, -30.30, 1),
    (71.58, -32.58, 1),
    (69.30, -35.98, 1),
    (68.50, -40.00, 1),
    (69.30, -44.02, 1),
    (71.58, -47.42, 1),
    (74.98, -49.70, 1),
    (79.00, -50.50, 1),
    (83.02, -49.70, 1),
    (86.42, -47.42, 1),
    (88.70, -44.02, 1),
    (89.50, -40.00, 1),
    (88.70, -35.98, 1),
    (86.42, -32.58, 1),
    (83.02, -30.30, 1),
    (79.00, -29.50, 1),
]

def inverse_kinematics(x, y):
    """(x, y) mm -> (servo1_deg, servo2_deg). Raises ValueError if unreachable."""
    c2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    if c2 < -1 or c2 > 1:
        raise ValueError("(%.1f, %.1f) is out of reach" % (x, y))
    t2 = ELBOW_SIGN * math.acos(c2)
    t1 = math.atan2(y, x) - math.atan2(L2 * math.sin(t2), L1 + L2 * math.cos(t2))
    s1 = (SERVO1_DIR * (math.degrees(t1) - THETA1_OFFSET)) % 360
    s2 = ((SERVO2_DIR * (math.degrees(t2) - THETA2_OFFSET)) % 360) / GEAR_RATIO
    if not (SERVO1_RANGE[0] <= s1 <= SERVO1_RANGE[1]) or \
       not (SERVO2_RANGE[0] <= s2 <= SERVO2_RANGE[1]):
        raise ValueError("(%.1f, %.1f) needs servo angles %.1f, %.1f (out of range)"
                         % (x, y, s1, s2))
    return s1, s2


class Servo:
    def __init__(self, pin):
        self.pwm = PWM(Pin(pin), freq=50)          # 50 Hz -> 20 ms period

    def write(self, angle):
        angle = max(0, min(180, angle))
        us = MIN_PULSE_US + (MAX_PULSE_US - MIN_PULSE_US) * angle / 180
        self.pwm.duty_u16(int(us * 65535 / 20000))  # pulse width as fraction of 20 ms

    def off(self):
        self.pwm.deinit()


servo1 = Servo(SERVO1_PIN)
servo2 = Servo(SERVO2_PIN)
led = Pin(LED_PIN, Pin.OUT) if LED_PIN is not None else None


def set_led(on):
    if led is not None:
        led.value(1 if on else 0)


def move_to(x0, y0, x1, y1):
    """Move in a straight line from (x0, y0) to (x1, y1) in small IK steps."""
    dist = math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)
    n = max(1, int(math.ceil(dist / STEP_MM)))
    for i in range(1, n + 1):
        t = i / n
        s1, s2 = inverse_kinematics(x0 + t * (x1 - x0), y0 + t * (y1 - y0))
        servo1.write(s1)
        servo2.write(s2)
        time.sleep_ms(STEP_DELAY_MS)


def check_points(points):
    """Check every point (and the steps between them) before anything moves."""
    prev = None
    for p in points:
        if prev is not None:
            dist = math.sqrt((p[0] - prev[0]) ** 2 + (p[1] - prev[1]) ** 2)
            n = max(1, int(math.ceil(dist / STEP_MM)))
            for i in range(1, n + 1):
                t = i / n
                inverse_kinematics(prev[0] + t * (p[0] - prev[0]), prev[1] + t * (p[1] - prev[1]))
        else:
            inverse_kinematics(p[0], p[1])
        prev = p


def run(points):
    check_points(points)                  # raises before moving if anything is unreachable
    set_led(False)

    # Go to the first point with the LED off
    x, y = points[0][0], points[0][1]
    s1, s2 = inverse_kinematics(x, y)
    print("start  (%6.2f, %6.2f) -> servo1 %5.1f, servo2 %5.1f" % (x, y, s1, s2))
    servo1.write(s1)
    servo2.write(s2)
    time.sleep_ms(1000)                   # let the arm get there from wherever it was
    time.sleep(START_DELAY_S)

    for i in range(1, len(points)):
        p = points[i]
        on = p[2] if len(p) > 2 else 1
        s1, s2 = inverse_kinematics(p[0], p[1])
        print("%3d    (%6.2f, %6.2f) -> servo1 %5.1f, servo2 %5.1f  LED %s"
              % (i, p[0], p[1], s1, s2, "on" if on else "off"))
        set_led(on)
        move_to(x, y, p[0], p[1])
        if not on:
            time.sleep_ms(SETTLE_MS)       # arrive and settle before the next lit segment
        x, y = p[0], p[1]

    set_led(False)
    print("done")


def direction_name(deg):
    """Absolute angle (CCW from +x) -> compass-style word."""
    names = ["RIGHT", "up-right", "UP", "up-left", "LEFT", "down-left", "DOWN", "down-right"]
    return names[int(((deg % 360) + 22.5) // 45) % 8]


def calibrate():
    for s1, s2 in [(0, 0), (90, 0), (90, 45)]:
        a1 = THETA1_OFFSET + SERVO1_DIR * s1                    # link 1 absolute angle
        a2 = a1 + THETA2_OFFSET + SERVO2_DIR * s2 * GEAR_RATIO  # link 2 absolute angle
        print("servo1 = %3d, servo2 = %3d : settings predict link 1 %s, link 2 %s"
              % (s1, s2, direction_name(a1), direction_name(a2)))
        servo1.write(s1)
        servo2.write(s2)
        time.sleep(4)


CALIBRATE = False        # True = run the pose check instead of drawing

try:
    if CALIBRATE:
        calibrate()
    else:
        run(POINTS)
finally:
    set_led(False)
    # Uncomment to let the servos go limp when finished:
    # servo1.off(); servo2.off()