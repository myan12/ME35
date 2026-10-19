# Read-only: GET the assembly's mate values from Onshape every second and print them.
# No servos, no motors. Drag the arm in Onshape and watch which values change.

import network, time, math, ubinascii
import secrets

try:
    import requests
except ImportError:
    import urequests as requests

DID = "774d39bdec33d0c04457313f"
WID = "e435f4bc1e9dbd83b50ffd84"
EID = "a52a0be965b84de14e97ad69"
URL = "https://cad.onshape.com/api/v10/assemblies/d/{}/w/{}/e/{}/matevalues".format(DID, WID, EID)
DELAY = 1   # seconds between reads

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
    raise RuntimeError("WiFi connection failed")
print("WiFi connected:", wlan.ifconfig()[0])
print("Reading:", URL)

# --- read loop
n = 0
try:
    while True:
        n += 1
        r = requests.get(URL, headers=HEADERS)
        try:
            if r.status_code != 200:
                print("Read", n, "- status", r.status_code, r.text[:200])
                mates = None
            else:
                mates = r.json()["mateValues"]
        finally:
            r.close()

        if mates is not None:
            print("===== read", n, "-", len(mates), "mates =====")
            for m in mates:
                print("Mate:", m.get("mateName"), "| type:", m.get("jsonType"),
                      "| featureId:", m.get("featureId"))
                # every numeric field, with angles also shown in degrees
                for k in sorted(m.keys()):
                    v = m[k]
                    if isinstance(v, (int, float)):
                        if k.startswith("rotation"):
                            print("   {:<14} {:>10.4f} rad  = {:>8.2f} deg".format(k, v, math.degrees(v)))
                        else:
                            print("   {:<14} {:>10.4f}".format(k, v))
                    elif k not in ("mateName", "featureId", "jsonType"):
                        print("   {:<14} {}".format(k, v))
        time.sleep(DELAY)
except KeyboardInterrupt:
    print("Stopped")