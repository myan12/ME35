from machine import Pin, PWM
import time

ELBOW_PIN = 5
STEPS = [0, 45, 90, 135, 180]   # servo angles to test

elbow = PWM(Pin(ELBOW_PIN), freq=50, duty_u16=0)

def deg2serv(theta):
    theta = max(0, min(180, theta))
    return int((theta / 180 * 2 + 0.5) * 65535 / 20)   # 0.5-2.5 ms pulse, same as follower.py

def go(angle):
    elbow.duty_u16(deg2serv(angle))
    print("Servo at {:5.1f} deg".format(angle))

try:
    # 1) step through the preset angles
    print("Step test: measure the elbow joint angle at each step.")
    for a in STEPS:
        go(a)
        input("  Measure the joint, then press Enter...")

    # 2) free test: type any servo angle
    print("\nType a servo angle (0-180) to move there, or 'q' to quit.")
    while True:
        s = input("servo angle> ").strip()
        if s.lower() == "q":
            break
        try:
            go(float(s))
        except ValueError:
            print("Enter a number 0-180 or 'q'")
except KeyboardInterrupt:
    pass
finally:
    elbow.duty_u16(0)
    elbow.deinit()
    print("Stopped")