# Onshape API connection test - no servos, no motors.
# Connects to WiFi, makes ONE GET for the assembly's mate values, and prints them.

import network, time, json, ubinascii
import secrets

try:
    import requests            
except ImportError:
    import urequests as requests

DID = "774d39bdec33d0c04457313f"
WID = "e435f4bc1e9dbd83b50ffd84"
EID = "a52a0be965b84de14e97ad69"

URL = "https://cad.onshape.com/api/v10/assemblies/d/{}/w/{}/e/{}/matevalues".format(DID, WID, EID)

token = ubinascii.b2a_base64("{}:{}".format(secrets.ACCESSKEY, secrets.SECRETKEY)).decode().strip()
HEADERS = {'Accept': 'application/json;charset=UTF-8;qs=0.09',
           'Content-Type': 'application/json',
           'Authorization': 'Basic ' + token}

# --- WiFi
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
if not wlan.isconnected():
    print("Connecting to WiFi...")
    wlan.connect(secrets.SSID, secrets.PWD)
    for _ in range(40):
        if wlan.isconnected():
            break
        time.sleep(0.5)
if not wlan.isconnected():
    raise RuntimeError("WiFi connection failed - check WIFI_SSID / WIFI_PASS")
print("WiFi connected:", wlan.ifconfig()[0])

# --- One API call
print("GET", URL)
r = requests.get(URL, headers=HEADERS)
print("Status:", r.status_code)

if r.status_code == 200:
    mates = r.json()["mateValues"]
    print("Found", len(mates), "mates:")
    for m in mates:
        print(" ", m.get("mateName"), "| featureId:", m.get("featureId"))
        print("   ", json.dumps(m))
    if not mates:
        print("Empty list - make sure EID is the Assembly tab and the mates are revolute.")
elif r.status_code == 401:
    print("Unauthorized - check ACCESSKEY / SECRETKEY in secrets.py")
elif r.status_code == 403:
    print("Forbidden - key lacks read permission, or no access to this document")
elif r.status_code == 404:
    print("Not found - check DID / WID / EID (EID must be the Assembly tab)")
else:
    print(r.text[:300])

r.close()