import serial
import time
import random

SERIAL_PORT = "/dev/serial0"  # or /dev/ttyAMA0
BAUD_RATE = 57600  # 3DR default

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

print("Transmitting...")

while True:
    data = f"Pi Data: {time.time()} Random={random.randint(0,100)}\n"
    ser.write(data.encode())
    print("Sent:", data.strip())
    time.sleep(1)
