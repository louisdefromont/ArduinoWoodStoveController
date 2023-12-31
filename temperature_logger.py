import os
import time
import requests

local_ip = "192.168.1.161" # Change this to the local IP address of your ESP8266

# Send get request to /api/temperature
def get_temperature():
    try:
        r = requests.get("http://" + local_ip + "/api/temperature")
        return r.text
    except:
        return "Error"
    
# Get temperature every 4 seconds and save it to temperature_log.csv along with the current date and time
def main():
    if not os.path.exists("temperature_log.csv"):
        with open("temperature_log.csv", "w") as f:
            f.write("Date Time,Temperature\n")
    while True:
        temperature = get_temperature()
        dateTime = time.strftime("%Y-%m-%d %H:%M:%S")
        if temperature != "Error":
            with open("temperature_log.csv", "a") as f:
                f.write(dateTime + "," + temperature + "\n")
        else:
            print("Error getting temperature")
        time.sleep(4)

if __name__ == "__main__":
    main()