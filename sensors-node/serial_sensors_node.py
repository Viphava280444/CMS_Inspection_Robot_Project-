import serial
import requests
import json
import time

# Update with your correct serial port
ser = serial.Serial("/dev/ttyACM1", 9600)  # Replace with your actual serial port
time.sleep(2)  # Wait for the serial connection to initialize

while True:
    if ser.in_waiting > 0:
        data = ser.readline().decode("utf-8", errors="ignore").strip()
        print(f"Raw data: {data}")  # Print raw data for debugging
        try:
            json_data = json.loads(data)
            response = requests.post("http://localhost:5001/data", json=json_data)
            print(
                f"Data sent: {json_data}, Response: {response.status_code}, Content: {response.content}"
            )
        except json.JSONDecodeError:
            print(f"Invalid JSON: {data}")

