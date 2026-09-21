#importing the libraries
import network
import urequests
import time
 
#setting up SSID and password 
 
SSID = "Tufts_Wireless" #use tufts_eecs
PASSWORD = "" #foundedin1883


#function definition 
def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print("Connecting to WiFi...")
        wlan.connect(SSID, PASSWORD)
        while not wlan.isconnected():
            time.sleep(0.5)
    print("Connected! IP address:", wlan.ifconfig()[0])
    return wlan