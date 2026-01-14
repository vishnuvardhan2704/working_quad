# KML Boundary Transmission Protocol - Quick Reference

## Architecture
```
┌─────────────────────┐         3DR Radio         ┌──────────────────────┐
│  GCS (Laptop)       │ ◄────────────────────────► │  RPI (On Drone)      │
│  tx_commands.py     │   Compressed Boundary      │  rx_commands.py      │
└─────────────────────┘   (200 bytes typical)      └──────────────────────┘
         │                                                     │
         │ 1. Read KML file                                   │ 4. Decompress
         │ 2. Extract boundary                                │ 5. Generate waypoints
         │ 3. Compress & send                                 │ 6. Upload to Pixhawk
         └────────────────────────────────────────────────────┘
```

## Why This Approach?

### ❌ What Doesn't Work:
- **Send full KML**: 3DR radio will corrupt XML format
- **Parse on GCS, send all waypoints**: 50 acres = 500+ waypoints = floods radio

### ✅ What Works:
- **Send boundary polygon only**: 4-8 corners = ~200 bytes compressed
- **Generate waypoints on RPI**: Uses existing `nidar/kml_parsing/` modules
- **Reliable transmission**: Chunked with progress tracking

## Protocol Specification

### Message Format

#### 1. START Command
```
KML:START:{size}:{altitude}:{pattern}
```
- `size`: Length of base64-encoded compressed data
- `altitude`: Flight altitude in meters AGL
- `pattern`: "curved" or "lawnmower"

Example: `KML:START:187:15:curved`

#### 2. DATA Chunks
```
KML:DATA:{chunk}
```
- `chunk`: 64 bytes of base64-encoded data
- Sent repeatedly until all data transmitted
- 100ms delay between chunks

Example: `KML:DATA:UEsDBBQAAAAIAMx...`

#### 3. END Command
```
KML:END
```
Triggers decompression and waypoint generation on RPI.

### Response Messages

From RPI → GCS:
- `[DRONE] KML: Ready to receive {size} bytes at {alt}m`
- `[DRONE] KML: Receiving... 25%`
- `[DRONE] KML: Receiving... 50%`
- `[DRONE] KML: Receiving... 75%`
- `[DRONE] KML: Received {size} bytes, processing...`
- `[DRONE] KML: Parsed {n} boundary points`
- `[DRONE] KML: Generating waypoints at {alt}m AGL...`
- `[DRONE] KML: Generated {n} waypoints`
- `[DRONE] KML: Uploading mission to Pixhawk...`
- `[DRONE] SUCCESS: Mission uploaded - {n} waypoints ready`
- `[DRONE] Ready: ARM -> TAKEOFF -> MODE:AUTO to begin survey`

## User Workflow

### Step 1: Upload Mission from GCS
```bash
$ python3 tx_commands.py

Enter command: KML:survey_area.kml,15

# GCS compresses and sends boundary
Compressing KML boundary...
Sending 0%... 25%... 50%... 75%... 100%
KML transmission complete

# RPI generates waypoints
[DRONE] KML: Ready to receive 187 bytes at 15.0m
[DRONE] KML: Receiving... 25%
[DRONE] KML: Receiving... 50%
[DRONE] KML: Receiving... 75%
[DRONE] KML: Received 187 bytes, processing...
[DRONE] KML: Parsed 5 boundary points
[DRONE] KML: Generated 47 waypoints
[DRONE] SUCCESS: Mission uploaded - 47 waypoints ready
[DRONE] Ready: ARM -> TAKEOFF -> MODE:AUTO to begin survey
```

### Step 2: Test Connection
```bash
Enter command: PING
[DRONE] PONG
```

### Step 3: Arm Drone
```bash
Enter command: ARM
[DRONE] Running preflight checks...
[DRONE] GPS: OK (16 satellites, fix=3)
[DRONE] Battery: 12.4V (100%)
[DRONE] EKF: OK
[DRONE] Armed
```

### Step 4: Takeoff
```bash
Enter command: TAKEOFF:15
[DRONE] Taking off to 15.0m...
[TELEM][INFO] MODE:GUIDED | ARM | Alt:2.3 | 12.4V | GPS:16
[TELEM][INFO] MODE:GUIDED | ARM | Alt:5.8 | 12.4V | GPS:16
[TELEM][INFO] MODE:GUIDED | ARM | Alt:11.2 | 12.3V | GPS:16
[TELEM][INFO] MODE:GUIDED | ARM | Alt:15.0 | 12.3V | GPS:16
[DRONE] Reached target altitude: 15.0m
```

### Step 5: Start Mission
```bash
Enter command: MODE:AUTO
[DRONE] Mode changed to AUTO
[TELEM][INFO] MODE:AUTO | ARM | Alt:15.0 | 12.3V | GPS:16
# Drone now follows uploaded waypoints autonomously
```

### Step 6: Monitor Progress
Telemetry updates every 2 seconds:
```
[TELEM][INFO] MODE:AUTO | ARM | Alt:15.1 | 12.2V | GPS:16
[TELEM][INFO] MODE:AUTO | ARM | Alt:15.0 | 12.2V | GPS:16
```

### Step 7: Return Home (when complete)
```bash
Enter command: RTL
[DRONE] Returning to launch
[TELEM][INFO] MODE:RTL | ARM | Alt:15.0 | 12.1V | GPS:16
```

## Competition Compliance

✅ **Rules Satisfied:**
- KML file uploaded/configured from GCS (not on drone)
- Mission triggered via GCS GUI command
- Drone processes data and executes autonomously
- No manual waypoint entry required

## Data Sizes

### Example: 50-acre field
- **KML file**: ~2-3 KB (with all XML formatting)
- **Boundary polygon**: 6-10 corner points
- **Compressed boundary**: ~150-250 bytes
- **Generated waypoints**: 200-500 waypoints
- **Transmission time**: ~5-10 seconds @ 57600 baud

## Error Handling

### If transmission fails:
```
[DRONE] ERROR: KML processing error: ...
```
**Solution**: Re-send from GCS (just run `KML:file,alt` again)

### If mission upload fails:
```
[DRONE] ERROR: Mission upload failed
```
**Solution**: Check Pixhawk connection, try again

### If waypoint count is wrong:
```
[DRONE] KML: Generated 0 waypoints
```
**Solution**: Check KML file has valid polygon boundary

## File Locations

### On GCS (Laptop):
- `tx_commands.py` - Command sender
- `missions/survey_area.kml` - KML files (or any location)

### On RPI (Drone):
- `rx_commands.py` - Command receiver
- `nidar/kml_parsing/path_planner.py` - Waypoint generation
- `nidar/mission/waypoint_mission.py` - Mission upload

## Implementation Status

### ✅ RPI Side (COMPLETED)
- [x] KML boundary reception buffer
- [x] Chunked data reception (KML:START, KML:DATA, KML:END)
- [x] Base64 + zlib decompression
- [x] Coordinate parsing
- [x] Waypoint generation using nidar modules
- [x] Mission upload to Pixhawk
- [x] Progress reporting

### ⏳ GCS Side (TO BE IMPLEMENTED)
- [ ] KML boundary extraction
- [ ] Compression (zlib + base64)
- [ ] Chunked transmission
- [ ] Progress reporting
- [ ] User command interface

**Use the `GCS_COPILOT_PROMPT.md` file to implement GCS side changes.**

## Troubleshooting

### Q: Transmission seems slow
**A**: Normal. 3DR radio @ 57600 baud + 100ms delays = ~10 seconds for 200 bytes

### Q: Data corruption during transmission
**A**: Increase chunk delay from 100ms to 200ms in GCS code

### Q: Waypoint count seems low
**A**: Adjust overlap parameter in path_planner (default 0.25 = 25% overlap)

### Q: Mission upload fails
**A**: Check Pixhawk is in GUIDED mode and armed before uploading

---

**Next Step**: Copy `GCS_COPILOT_PROMPT.md` to your GCS laptop and use it with Copilot to implement the transmission code in `tx_commands.py`.
