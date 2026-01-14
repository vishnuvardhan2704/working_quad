#!/usr/bin/env python3
"""Quick battery test for RadioLink Pix6"""
from dronekit import connect
import time

print("Connecting to /dev/ttyACM1...")
v = connect('/dev/ttyACM1', baud=115200, wait_ready=True, timeout=30)
print(f"Connected! Firmware: {v.version}")

print("\n=== BATTERY READING ===")
for i in range(10):
    bat = v.battery
    voltage = bat.voltage if bat else 0
    current = bat.current if bat else 0
    level = bat.level if bat else 0
    print(f"[{i+1}] V={voltage}V, A={current}A, Level={level}%")
    time.sleep(1)

print("\n=== PARAMETERS ===")
params = ['BATT_MONITOR', 'BATT_VOLT_PIN', 'BATT_CURR_PIN', 'BATT_VOLT_MULT']
for p in params:
    print(f"{p} = {v.parameters.get(p, 'N/A')}")

v.close()
print("\nDone")
