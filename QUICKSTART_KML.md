# 🚀 QUICK START - KML Mission Upload

## TL;DR - What You Need to Do

### On Your GCS Laptop (Ground Station)

1. **Open the prompt file:**
   ```bash
   cat GCS_COPILOT_PROMPT.md
   ```

2. **Open VS Code with your GCS project:**
   ```bash
   code tx_commands.py
   ```

3. **Open GitHub Copilot Chat and paste this:**
   ```
   I need to update tx_commands.py to transmit KML survey missions to the drone via 3DR radio. 
   
   [Then paste the entire contents of GCS_COPILOT_PROMPT.md]
   ```

4. **Test it:**
   ```bash
   python3 tx_commands.py
   
   # Enter commands:
   KML:survey_area.kml,15
   PING
   ARM
   TAKEOFF:15
   MODE:AUTO
   ```

---

## What Changed

### RPI Side ✅ (Already Done)
Your drone can now:
- Receive compressed KML boundaries from GCS
- Decompress and parse polygon coordinates
- Generate hundreds of waypoints automatically
- Upload mission to Pixhawk
- Report progress back to GCS

### GCS Side ⏳ (Your Task)
You need to add code that:
- Reads KML file
- Extracts polygon boundary (just the corner points)
- Compresses it (zlib + base64)
- Sends it in chunks to the drone
- Displays progress

**The prompt in `GCS_COPILOT_PROMPT.md` tells Copilot exactly how to do this.**

---

## Why This Solves Your Problem

| Your Concern | The Solution |
|--------------|--------------|
| "Rules say mission must be configured at GCS" | ✅ You upload KML from GCS, not from drone |
| "LORA will mess up full KML format" | ✅ We only send the boundary (5 points = 200 bytes) |
| "50 acres = thousands of waypoints floods radio" | ✅ Waypoints generated on drone, not transmitted |
| "Need to trigger via GCS GUI" | ✅ You type `KML:file.kml,15` then `MODE:AUTO` |

---

## The Protocol (Simplified)

```
GCS                          3DR Radio                     RPI
 │                              │                           │
 │  KML:START:187:15:curved    │                           │
 ├─────────────────────────────►──────────────────────────►│ [Prepare buffer]
 │                              │                           │
 │  KML:DATA:UEsDBBQAAA...      │                           │
 ├─────────────────────────────►──────────────────────────►│ [Receiving 25%]
 │  KML:DATA:AIAMx9VzdH...      │                           │
 ├─────────────────────────────►──────────────────────────►│ [Receiving 50%]
 │  KML:DATA:RwZXIucnBp...      │                           │
 ├─────────────────────────────►──────────────────────────►│ [Receiving 75%]
 │                              │                           │
 │  KML:END                     │                           │
 ├─────────────────────────────►──────────────────────────►│ [Processing...]
 │                              │                           │
 │                              │  [DRONE] Generated 47 WPs │
 │◄─────────────────────────────◄──────────────────────────┤
 │                              │  [DRONE] Mission uploaded │
 │◄─────────────────────────────◄──────────────────────────┤
```

Only **200 bytes** transmitted instead of **5000+ waypoint coordinates**!

---

## Typical Mission Sequence

```bash
# Step 1: Upload mission
KML:survey_area.kml,15
→ [DRONE] SUCCESS: Mission uploaded - 47 waypoints ready

# Step 2: Test connection
PING
→ [DRONE] PONG

# Step 3: Arm
ARM
→ [DRONE] Running preflight checks...
→ [DRONE] Armed

# Step 4: Takeoff
TAKEOFF:15
→ [DRONE] Taking off to 15.0m...
→ [DRONE] Reached target altitude: 15.0m

# Step 5: Start autonomous mission
MODE:AUTO
→ [DRONE] Mode changed to AUTO
→ Drone follows waypoints automatically

# Step 6: Monitor (telemetry updates every 2s)
[TELEM][INFO] MODE:AUTO | ARM | Alt:15.1 | 12.3V | GPS:16

# Step 7: Return home when done
RTL
→ [DRONE] Returning to launch
```

---

## Files You Need

### On RPI (Drone) - Already There ✅
- `rx_commands.py` - Updated with KML reception
- `nidar/kml_parsing/path_planner.py` - Generates waypoints
- `nidar/mission/waypoint_mission.py` - Uploads to Pixhawk

### On GCS (Laptop) - Need to Copy 📋
1. `GCS_COPILOT_PROMPT.md` ← **USE THIS WITH COPILOT**
2. `tx_commands.py` ← **THIS NEEDS TO BE UPDATED**
3. `missions/survey_area.kml` ← Your KML files

---

## Testing Checklist

After implementing GCS side:

- [ ] Can read KML file
- [ ] Extracts boundary coordinates
- [ ] Compresses to ~200 bytes
- [ ] Sends via `KML:file,alt` command
- [ ] Shows progress (0%, 25%, 50%, 75%, 100%)
- [ ] Drone responds with waypoint count
- [ ] Can ARM, TAKEOFF, MODE:AUTO
- [ ] All existing commands still work (PING, STATUS, etc.)

---

## If Something Goes Wrong

### "KML file not found"
- Check file path (try absolute path or put in current directory)
- Make sure file exists: `ls -la missions/survey_area.kml`

### "No polygon found in KML"
- Open KML in text editor
- Make sure it has `<Polygon>` and `<coordinates>` tags
- Your `survey_area.kml` looks good ✓

### "Mission upload failed"
- Check Pixhawk connection on RPI
- Make sure drone is in GUIDED mode
- Check `rx_commands.py` logs on RPI

### "Transmission seems stuck"
- Normal - takes 5-10 seconds for full transmission
- Watch for progress updates (25%, 50%, 75%)
- If no updates after 30s, try again

---

## Competition Compliance ✅

**Rule**: *"To fly autonomously must only upload/configure the KML file in the Command and Control Station and trigger a launch command via the GCS GUI"*

**How we comply**:
1. ✅ KML file is at GCS (your laptop)
2. ✅ You configure mission from GCS: `KML:survey_area.kml,15`
3. ✅ You trigger launch from GCS: `ARM` → `TAKEOFF` → `MODE:AUTO`
4. ✅ Drone processes data and executes autonomously

**Judges will see**:
- You select KML file on your laptop
- You click "Upload Mission" (or type the command)
- You click "ARM" → "TAKEOFF" → "START MISSION"
- Drone takes off and follows the survey pattern
- All from your ground station laptop ✓

---

## Next Action

**Copy this to your GCS laptop and give the prompt to Copilot:**

```bash
# On RPI (copy file to GCS)
scp GCS_COPILOT_PROMPT.md user@your-gcs-laptop:/path/to/project/

# On GCS (open in VS Code)
code tx_commands.py

# In Copilot Chat, paste GCS_COPILOT_PROMPT.md
```

---

**That's it!** The RPI side is done. Just implement GCS side using the prompt and you're ready to fly! 🚁
