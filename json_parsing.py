#Examples of dictionaries
# data = {"key":"value"} 
# print(data["key"])
# 
# data1 = {"key1" : "value1", "key2" : "value2"}
# print(data1["key1"])
# 
# data2 = [{"key1" : "A", "key2" : "B"}, {"key1" : "C", "key2" : "D"}]
# print(data2[0]["key1"])

# student = {
#     "name": "Alex",
#     "score": 92,
#     "passed": True
# }
# 
# print(student["name"])
# print(student["score"])
# print(student["passed"])

# robot = {
#     "name": "Rover",
#     "sensors": ["distance", "temperature", "light"]
# }
# 
# print(robot["sensors"][0])  # distance
# print(robot["sensors"][2])  # light

# motors = [
#     {"name": "left", "speed": 50},
#     {"name": "right", "speed": 60}
# ]
# 
# for motor in motors:
#     print(motor["name"])
#     print(motor["speed"])

# sensor = {
#     "type": "temperature",
#     "value": 24.5,
#     "unit": "fahrenheit"
# }
# 
# if "value" in sensor:
#     print(sensor["value"])
# 
# if "unit" in sensor:
#     print(sensor["unit"])
# else:
#     print("No unit was provided")

# sensor = {
#     "type": "distance",
#     "value": 35
# }
# 
# print(sensor.get("value"))
# print(sensor.get("unit", "cm"))