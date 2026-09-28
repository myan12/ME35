from machine import Pin, I2C
i2c = I2C(0, sda=Pin(21), scl=Pin(22), freq=400000)
print(i2c.scan())

import time

#importing color sensor library
import veml6040
# create color sensor object
sensor = veml6040.VEML6040(i2c)
# sets how long the sensor collects light for each reading
# think of camera shutter speed
sensor.set_integration_time(veml6040.IT_160MS)
# only takes readings when you force it by button
# instead of auto mode where it refreshes reading every 
sensor.set_force_mode()               
IT_MS = 160

btn = Pin(34, Pin.IN, Pin.PULL_UP)                
DEBOUNCE_MS = 200
last_press = 0
btn_pressed = False

# button handler to set button flag to true for main loop
def button_handler(pin):
    global btn_pressed, last_press
    now = time.ticks_ms()
    if time.ticks_diff(now, last_press) > DEBOUNCE_MS:
        last_press = now
        btn_pressed = True  
        
btn.irq(trigger=Pin.IRQ_FALLING, handler=button_handler)

while True:
    if btn_pressed:
        btn_pressed = False
        sensor.trigger_measurement()           # start one fresh measurement
        time.sleep_ms(IT_MS + 10)              # wait for it to finish taking data
        r, g, b, w = sensor.read_rgbw()
        print("R:", r, "G:", g, "B:", b, "W:", w)
    time.sleep_ms(10)                  
    
    


