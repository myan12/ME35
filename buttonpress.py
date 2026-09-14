from machine import Pin
from time import ticks_ms, ticks_diff
import neopixel

btn = Pin(34, Pin.IN, Pin.PULL_UP)
lights = neopixel.NeoPixel(Pin(15),2)

DEBOUNCE_MS = 200
last_press = 0
led = False

def button_handler(pin):
    global last_press, led
    now = ticks_ms()
    if ticks_diff(now, last_press) > DEBOUNCE_MS:
        if (pin.value() == 0):
            last_press = now
            if (led):
                lights[0] = (20,0,20)
                lights[1] = (0,0,0)
            else:
                lights[1] = (20,0,20)
                lights[0] = (0,0,0)
            lights.write()
            led = not led
            print("button pressed")

btn.irq(trigger=Pin.IRQ_FALLING, handler=button_handler)