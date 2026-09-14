import urequests

# API_URL = "https://v6.exchangerate-api.com/v6/251cc379a5138503a3e4c46e/latest/USD"
# reply = urequests.get(API_URL)
# data = reply.json()
# reply.close()
# 
# rates = data['conversion_rates']
# 
# eur = rates['EUR']
# gbp = rates['GBP']
# jpy = rates['JPY']
# cad = rates['CAD']
# 
# print("1 USD =", eur, "EUR")
# print("1 USD =", gbp, "GBP")
# print("1 USD =", jpy, "JPY")
# print("1 USD =", cad, "CAD")

DATE_URL = "https://timeapi.io/api/time/current/zone?timeZone=America/New_York"
reply = urequests.get(DATE_URL)
data = reply.json()
reply.close()

time = data['time']

print(time)

 #ON REPL you can type wlan.isconnected() hit ENTER. it will return True
