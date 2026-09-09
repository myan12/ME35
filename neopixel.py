import neopixel #importing the library
import time
from machine import Pin # another way of importing a library
for i in range(5):
    lights = neopixel.NeoPixel(Pin(15),2) # 0 is the Pin for neopixel and 4 is the number of lights
    lights[0] = (20,0,20) # set the color of 0th light to purple
    lights[1] = (255,0,0)
    lights.write()
    time.sleep(2)
    lights[0] = (255, 119, 130)
    lights[1] = (0, 255, 0)
    lights.write()
    time.sleep(2)
lights[0] = (0, 0, 0)
lights[1] = (0, 0, 0)
lights.write()

