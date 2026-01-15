# GUDDABAL Delivery Drone System

## Overview

The Guddabal Delivery Drone is an autonomous package delivery system that receives target coordinates from a Ground Control Station (GCS), flies to the location, detects a human target using computer vision, precisely centers over them, and releases a package using a stepper motor mechanism.

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                      GROUND CONTROL STATION (GCS)                    │
│                                                                      │
│   Sends: GOTO:lat,lon,alt    ◄────────────────────────────────────┐ │
│   Receives: Telemetry, Status, Detection Logs                     │ │
└────────────────────────────────────────────────────────────────────┘ │
                              │                                        │
                              │ LoRa Radio                             │
                              ▼                                        │
┌─────────────────────────────────────────────────────────────────────┐│
│                       DELIVERY DRONE (Raspberry Pi)                 ││
│                                                                     ││
│  ┌─────────────┐  ┌──────────────┐  ┌─────────────────────────────┐││
│  │   main.py   │  │ visual_servo │  │  StepperMotorController     │││
│  │  (Command   │──│   .py        │──│  (Package Release)          │││
│  │   Handler)  │  │ (Detection)  │  │                             │││
│  └─────────────┘  └──────────────┘  └─────────────────────────────┘││
│         │                │                        │                 ││
│         ▼                ▼                        ▼                 ││
│  ┌─────────────┐  ┌──────────────┐  ┌─────────────────────────────┐││
│  │  Pixhawk    │  │  RealSense   │  │  NEMA 17 Stepper Motor      │││
│  │  Pix6       │  │  D435        │  │  + A4988/DRV8825 Driver     │││
│  │  (Flight    │  │  (Camera)    │  │                             │││
│  │  Controller)│  │              │  │                             │││
│  └─────────────┘  └──────────────┘  └─────────────────────────────┘││
│                                                                     ││
└─────────────────────────────────────────────────────────────────────┘│
                              │                                        │
                              │ Logs sent back via LoRa                │
                              └────────────────────────────────────────┘
```

---

## Hardware Components

| Component | Model | Purpose |
|-----------|-------|---------|
| **Flight Controller** | RadioLink Pix6 | ArduCopter 4.6.3, flight control |
| **Companion Computer** | Raspberry Pi | Runs main.py, visual_servo, motor control |
| **Camera** | Intel RealSense D435 | Human detection, depth sensing |
| **Detection Model** | YOLOv8 ONNX | Human detection (`best.onnx`) |
| **Stepper Motor** | NEMA 17 | Package release mechanism |
| **Motor Driver** | A4988/DRV8825 | Stepper motor driver |
| **Radio** | LoRa | GCS communication |

---

## GPIO Wiring (Stepper Motor)

```
Raspberry Pi (BCM)          A4988/DRV8825 Driver
─────────────────           ────────────────────
GPIO 18  ───────────────►   STEP
GPIO 23  ───────────────►   DIR (Direction)
GPIO 24  ───────────────►   ENABLE (Active LOW)
3.3V     ───────────────►   VDD
GND      ───────────────►   GND

Driver Configuration:
- MS1, MS2, MS3 → GND (Full step mode)
- RESET & SLEEP → Shorted together
- VMOT → Motor power supply (8-35V)
```

---

## Flight Parameters

### Altitudes

| Phase | Altitude | Description |
|-------|----------|-------------|
| **Approach** | 15.0 m | Initial approach to target GPS |
| **Delivery** | 6.0 m | Final hover altitude for package release |
| **Descent Step** | 1.0 m | Incremental descent while re-centering |

### Speeds

| Parameter | Value | Description |
|-----------|-------|-------------|
| **Waypoint Groundspeed** | 5.0 m/s | Speed when flying to target |
| **Max Centering Velocity** | 1.0 m/s | Maximum horizontal adjustment speed |
| **Min Centering Velocity** | 0.2 m/s | Fine adjustment speed |
| **Descent Rate** | 0.5 m/s | Vertical descent speed |

### Timeouts

| Parameter | Value | Description |
|-----------|-------|-------------|
| **Hover Time** | 5.0 s | Time to hover after centering |
| **Centering Timeout** | 60 s | Max time to center over target |
| **Detection Timeout** | 30 s | Max time to detect human |
| **Waypoint Timeout** | 120 s | Max time to reach waypoint |

---

## Stepper Motor Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| **Microstep Mode** | 1 (Full Step) | No microstepping |
| **Steps per Revolution** | 200 | 1.8° per step |
| **Release Rotation** | 180° CW | Clockwise to release package |
| **Release Steps** | 100 | (180/360) × 200 = 100 steps |
| **Step Delay** | 3 ms | Delay between steps |
| **Total Release Time** | ~0.3 s | 100 steps × 3ms |

---

## Delivery Flow Sequence

```
┌────────────────────────────────────────────────────────────────────┐
│ 1. GCS sends: GOTO:lat,lon,alt                                     │
└────────────────────────────────────────────────────────────────────┘
                                   │
                                   ▼
┌────────────────────────────────────────────────────────────────────┐
│ 2. Drone flies to target GPS at 15m altitude @ 5 m/s               │
│    → Logs: "Flying to delivery location..."                        │
└────────────────────────────────────────────────────────────────────┘
                                   │
                                   ▼
┌────────────────────────────────────────────────────────────────────┐
│ 3. Arrive at location, start camera detection                      │
│    → YOLOv8 searches for human in frame                            │
│    → Timeout: 30 seconds                                           │
└────────────────────────────────────────────────────────────────────┘
                                   │
                          ┌───────┴────────┐
                          ▼                ▼
                   ┌──────────┐      ┌──────────┐
                   │ DETECTED │      │ TIMEOUT  │
                   └──────────┘      └──────────┘
                          │                │
                          │                ▼
                          │         Return to Home
                          ▼
┌────────────────────────────────────────────────────────────────────┐
│ 4. COARSE CENTERING at 15m                                         │
│    → Threshold: 50 pixels from center                              │
│    → Speed: up to 1.0 m/s                                          │
│    → Logs: "HUMAN DETECTED at lat,lon,alt"                         │
└────────────────────────────────────────────────────────────────────┘
                                   │
                                   ▼
┌────────────────────────────────────────────────────────────────────┐
│ 5. DESCEND to 6m in 1m steps                                       │
│    → Re-center at each altitude                                    │
│    → Descent rate: 0.5 m/s                                         │
│    15m → 14m → 13m → 12m → 11m → 10m → 9m → 8m → 7m → 6m           │
└────────────────────────────────────────────────────────────────────┘
                                   │
                                   ▼
┌────────────────────────────────────────────────────────────────────┐
│ 6. FINE CENTERING at 6m                                            │
│    → Threshold: 20 pixels from center                              │
│    → Speed: 0.2 - 1.0 m/s                                          │
└────────────────────────────────────────────────────────────────────┘
                                   │
                                   ▼
┌────────────────────────────────────────────────────────────────────┐
│ 7. HOVER for 5 seconds                                             │
│    → Stable position over target                                   │
└────────────────────────────────────────────────────────────────────┘
                                   │
                                   ▼
┌────────────────────────────────────────────────────────────────────┐
│ 8. RELEASE PACKAGE                                                 │
│    → Enable stepper motor                                          │
│    → Rotate 180° clockwise (100 steps @ 3ms)                       │
│    → Disable motor                                                 │
│    → Logs: "PACKAGE RELEASED at lat,lon,6.0m"                      │
└────────────────────────────────────────────────────────────────────┘
                                   │
                                   ▼
┌────────────────────────────────────────────────────────────────────┐
│ 9. DELIVERY COMPLETE                                               │
│    → Logs: "DELIVERY COMPLETE"                                     │
│    → Drone awaits next command or RTH                              │
└────────────────────────────────────────────────────────────────────┘
```

---

## Key Files

| File | Purpose |
|------|---------|
| `main.py` | Main drone controller, command handler, GCS communication |
| `visual_servo.py` | Visual servoing, human detection, motor control |
| `yolo_detector.py` | YOLOv8 ONNX inference for human detection |
| `test_stepper_motor.py` | Standalone motor testing script |
| `best.onnx` | YOLOv8 trained model for human detection |
| `tx_commands.py` / `rx_commands.py` | LoRa radio communication |

---

## GCS Commands

| Command | Description |
|---------|-------------|
| `GOTO:lat,lon,alt` | Fly to coordinates and execute delivery |
| `ARM` | Arm the drone |
| `DISARM` | Disarm the drone |
| `TAKEOFF:alt` | Take off to specified altitude |
| `LAND` | Land at current position |
| `RTH` | Return to home/launch position |
| `HOLD` | Hold current position (loiter) |
| `ABORT` | Emergency abort - immediate land |

---

## GCS Log Messages

During delivery, these messages are sent back to GCS:

```
"Flying to delivery location: lat,lon at 15.0m"
"HUMAN DETECTED at 12.345678,98.765432,15.0m"
"Centering over target..."
"Descending to 6.0m..."
"PACKAGE RELEASED at 12.345678,98.765432,6.0m"
"DELIVERY COMPLETE"
```

---

## Testing the Stepper Motor

### Interactive Mode
```bash
python3 test_stepper_motor.py
```

### Command Line Options
```bash
# Test package release (180° clockwise)
python3 test_stepper_motor.py --release

# Test retract (180° counter-clockwise)
python3 test_stepper_motor.py --retract

# Full delivery cycle (release + retract)
python3 test_stepper_motor.py --cycle

# Custom degrees
python3 test_stepper_motor.py --degrees 90

# Custom degrees counter-clockwise
python3 test_stepper_motor.py --degrees 90 --ccw

# Custom steps
python3 test_stepper_motor.py --step 50
```

### Test from Visual Servo Module
```python
from visual_servo import StepperMotorController

motor = StepperMotorController(logger=print)
motor.release_package()  # 180° CW, 100 steps
```

---

## Camera Configuration

### Intel RealSense D435

| Parameter | Value |
|-----------|-------|
| **Resolution** | 640 × 480 |
| **Horizontal FOV** | 87° |
| **Vertical FOV** | 58° |
| **Depth Range** | 0.1m - 10m |

### Detection Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| **Coarse Threshold** | 50 px | Centering tolerance at high altitude |
| **Fine Threshold** | 20 px | Centering tolerance at delivery altitude |
| **Frame Center** | (320, 240) | Center of 640×480 frame |

---

## Safety Features

1. **Detection Timeout**: If no human detected in 30s, abort delivery
2. **Centering Timeout**: If can't center in 60s, abort delivery
3. **Waypoint Timeout**: If can't reach waypoint in 120s, abort
4. **Motor Lazy Init**: Motor GPIO only activated during delivery, not before
5. **Motor Auto-Disable**: Motor disabled immediately after release

---

## Failsafe Conditions

| Condition | Action |
|-----------|--------|
| **RC Link Loss** | Return to Home (RTH) |
| **GCS Link Loss** | Continue mission or RTH |
| **Low Battery** | RTH when voltage critical |
| **Geofence Breach** | RTH |
| **Detection Failed** | Abort delivery, hold position |

---

## Running the Delivery Drone

### Start Main Service
```bash
python3 main.py
```

### Or via systemd service
```bash
sudo systemctl start quadtest.service
```

### View Logs
```bash
./view_logs.sh
# or
./view_session_logs.sh
```

---

## Troubleshooting

### Motor Not Rotating
1. Check GPIO wiring (18=STEP, 23=DIR, 24=ENABLE)
2. Verify motor power supply connected to driver VMOT
3. Check ENABLE pin is going LOW when activated
4. Run `python3 test_stepper_motor.py` to test standalone

### Motor Rotating Wrong Amount
1. Check `MICROSTEP_MODE` matches driver MS1/MS2/MS3 pins
2. If MS1/MS2/MS3 all grounded → Full step (MICROSTEP_MODE=1)
3. If only MS1 high → Half step (MICROSTEP_MODE=2)

### Human Not Detected
1. Ensure `best.onnx` model file exists
2. Check camera is connected: `rs-enumerate-devices`
3. Verify lighting conditions are adequate
4. Test with `python3 yolo_detector.py`

### Drone Not Responding to GOTO
1. Check LoRa radio connection
2. Verify GPS lock (>6 satellites)
3. Ensure drone is armed and in GUIDED mode
4. Check main.py logs for errors

---

## Version History

| Date | Version | Changes |
|------|---------|---------|
| 2026-01-15 | 1.0 | Initial delivery system with stepper motor |
| 2026-01-14 | 0.9 | Visual servo integration |
| 2026-01-13 | 0.8 | GOTO command delivery flow |

---

## Contact

For issues or questions about the Guddabal Delivery Drone system, check the logs in `/home/dart/quadtest/logs/sessions/` or review the code documentation.
