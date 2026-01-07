# SCOUT Command Update - Auto Camera & AUTO Mode

## Changes Made

### 1. **Camera Auto-Start on SCOUT Command**
   - Camera now automatically initializes when you send `SCOUT` or `SCOUT:KML:file,alt`
   - No need to manually send `DETECT:START` anymore

### 2. **AUTO Mode Auto-Switch**
   - If drone is already armed and airborne, SCOUT automatically switches to AUTO mode
   - Mission starts executing immediately
   - If on ground, you'll get instructions to ARM and TAKEOFF first

### 3. **Comprehensive Camera Status Logging**
   - Detailed camera initialization logs show exactly what's happening
   - Logs show which camera device is being used (/dev/video2, /dev/video4, etc.)
   - Clear "CAMERA: ONLINE" or "CAMERA: FAILED" status messages
   - Both RX (drone) and TX (ground station) will see camera status

## New SCOUT Workflow

### Ground Start (Recommended)
```
1. CMD> scout
   → Uploads mission waypoints
   → Starts camera and detection
   → Message: "SCOUT: Mission ready. Send ARM and TAKEOFF to begin."
   → Message: "Camera: ON" (or "FAILED" if error)

2. CMD> arm
   → Drone arms

3. CMD> takeoff:5
   → Drone takes off to 5 meters
   → AUTO mode engages automatically
   → Mission starts executing
   → Camera recording active
```

### Air Start (Already Flying)
```
1. [Already armed and at altitude]

2. CMD> scout
   → Uploads mission waypoints
   → Starts camera and detection
   → Message: "SCOUT: Starting AUTO mode with X waypoints"
   → Message: "Camera: ON"
   → AUTO mode activated immediately
   → Mission executing
```

## Camera Status Messages

You'll now see these messages in your logs:

### RX Side (Drone Logs)
```
[INFO] SCOUT: Auto-starting detection with camera recording...
[INFO] Attempting to initialize USB camera (OpenCV)...
[SUCCESS] ✓ USB Camera initialized on /dev/video2
[SUCCESS] ✓ Detection system ACTIVE with USB OpenCV camera
```

### TX Side (Ground Station)
```
CAMERA: USB /dev/video2 ONLINE
DETECTION: ACTIVE (USB OpenCV recording)
Camera: ON
```

### If Camera Fails
```
RX: [ERROR] ✗ Camera initialization FAILED - tried: /dev/video2, /dev/video4, /dev/video0, /dev/video1
TX: ERROR: Camera not found (tried /dev/video2, /dev/video4, /dev/video0, /dev/video1)
TX: Camera: FAILED
```

## Mission Progress

During AUTO mode mission:
```
[INFO] Switching to AUTO mode for mission...
[SUCCESS] AUTO mode activated - ArduPilot executing mission
[TX] AUTO mode active - mission started
[TX] Waypoint 1/15
[TX] Waypoint 2/15
...
[TX] Mission waypoints complete!
[TX] SCOUT COMPLETE! Total detections: 3
```

## Troubleshooting

### Camera Not Starting
- Check which camera devices exist: `ls -la /dev/video*`
- RealSense camera takes priority if available
- Falls back to USB cameras on /dev/video2, /dev/video4, /dev/video0, /dev/video1
- Logs will show exactly which devices were tried

### Mission Not Starting
- If on ground: ARM and TAKEOFF first
- If airborne but not starting: Check logs for AUTO mode switch confirmation
- Abort anytime with: `scout:stop` or `mode:rtl`

## Testing

To verify camera is working:
```bash
# Test camera detection
ls -la /dev/video*

# View live session logs
./view_session_logs.sh today

# Or restart service and monitor
sudo systemctl restart lora
sudo journalctl -u lora -f
```
