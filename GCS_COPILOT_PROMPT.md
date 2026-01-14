# GCS Side: KML Boundary Transmission Implementation

## Context
I need to update the GCS command sender (`tx_commands.py`) to transmit KML survey missions to the drone via 3DR radio. The drone side (RPI) has been updated to receive compressed KML boundary data, generate waypoints locally, and upload the mission to the Pixhawk.

## Requirements

### 1. Add KML Boundary Compression Function
Create a function that extracts and compresses polygon boundary coordinates from a KML file:

```python
def compress_kml_boundary(kml_file):
    """
    Extract and compress polygon boundary coordinates from KML file.
    
    Args:
        kml_file: Path to KML file
        
    Returns:
        Base64-encoded compressed coordinate string
        
    Process:
        1. Parse KML XML to extract <Polygon><coordinates> text
        2. Compress using zlib
        3. Base64 encode for safe transmission over radio
        4. Return encoded string
    """
```

Use `xml.etree.ElementTree` (avoid external dependencies like lxml if possible).

### 2. Add KML Mission Upload Method
Add a new method to the `DroneCommandSender` class:

```python
def send_kml_mission(self, kml_file, altitude, pattern="curved"):
    """
    Send compressed KML boundary + parameters to drone.
    
    Protocol:
        1. Send: "KML:START:{data_size}:{altitude}:{pattern}"
        2. Wait 200ms for drone to prepare
        3. Send boundary data in 64-byte chunks as "KML:DATA:{chunk}"
        4. Wait 100ms between chunks (respect 3DR radio buffer)
        5. Send: "KML:END" to trigger processing
        
    Args:
        kml_file: Path to KML fileerate waypoints locally, and upload the mission to the Pixhawk.

        altitude: Flight altitude in meters AGL
        pattern: "curved" or "lawnmower" (default: "curved")
    """
```

**Important**: 
- Chunk size should be 64 bytes for 3DR radio buffer safety
- Add 100ms delay between chunks
- Print progress updates to user (e.g., "Sending 25%...")

### 3. Update Interactive Menu
Update the `run_interactive()` method to:

1. **Add new command**: `KML:filename,altitude` 
   - Example: `KML:survey_area.kml,15`
   - This should call `send_kml_mission()`
   
2. **Update help text** to show:
   ```
   === AUTONOMOUS MISSION ===
   KML:file,alt         - Upload KML boundary for mission (e.g. KML:area.kml,15)
   
   === FLIGHT SEQUENCE (after KML upload) ===
   PING                 - Test connection
   ARM                  - Arm motors
   TAKEOFF:15           - Takeoff to 15m
   MODE:AUTO            - Start uploaded mission
   RTL                  - Return to launch
   LAND                 - Land immediately
   ABORT                - Emergency abort
   ```

3. **Keep existing commands** working (PING, ARM, TAKEOFF, MODE, etc.)

4. **Deprecation note**: Mark old `SCOUT` command as deprecated:
   ```
   SCOUT               - [DEPRECATED] Use KML:file,alt instead
   ```

### 4. User Workflowerate waypoints locally, and upload the mission to the Pixhawk.

The intended workflow is:
```
1. User enters: KML:survey_area.kml,15
   → GCS sends compressed boundary to drone
   → Drone generates waypoints and uploads mission
   
2. User enters: PING
   → Verify connection
   
3. User enters: ARM
   → Arm motors (with preflight checks)
   
4. User enters: TAKEOFF:15
   → Takeoff to 15 meters
   
5. User enters: MODE:AUTO
   → Drone starts autonomous survey mission
```

## Implementation Notes

- **File path handling**: Support both absolute paths and relative paths (check current directory first, then `/home/dart/quadtest/missions/`)
- **Error handling**: Check if KML file exists before attempting to send
- **User feedback**: Print status messages at each step:
  - "Compressing KML boundary..."
  - "Sending 0%... 25%... 50%... 75%... 100%"
  - "KML transmission complete, waiting for drone response..."
- **Receive drone responses**: Display all `[DRONE]` responses from the RPI

## Example Code Structure

```python
class DroneCommandSender:
    # ... existing code ...
    
    def compress_kml_boundary(self, kml_file):
        """Extract and compress KML boundary."""
        import xml.etree.ElementTree as ET
        import base64
        import zlib
        
        # Parse KML
        tree = ET.parse(kml_file)
        root = tree.getroot()
        
        # Find coordinates (handle KML namespace)
        ns = {'kml': 'http://www.opengis.net/kml/2.2'}
        coords = root.find('.//kml:Polygon//kml:coordinates', ns)
        
        if coords is None:
            # Try without namespace
            coords = root.find('.//Polygon//coordinates')
        
        if coords is None:
            raise ValueError("No polygon found in KML")
        
        coord_text = coords.text.strip()
        
        # Compress and encode
        compressed = zlib.compress(coord_text.encode('utf-8'))
        encoded = base64.b64encode(compressed).decode('ascii')
        
        return encoded
    
    def send_kml_mission(self, kml_file, altitude, pattern="curved"):
        """Send KML boundary to drone."""
        # ... implementation ...
    
    def run_interactive(self):
        # ... update command parsing ...
        if cmd.startswith("KML:"):
            # Parse KML:file,altitude
            params = cmd.split(":", 1)[1].split(",")
            kml_file = params[0]
            altitude = float(params[1]) if len(params) > 1 else 5.0
            self.send_kml_mission(kml_file, altitude)
        # ... keep other commands ...
```

## Testing
After implementation, test with:
```bash
# On GCS terminal
python3 tx_commands.py

# Enter commands:
KML:survey_area.kml,15
PING
ARM
TAKEOFF:15
MODE:AUTO
```

## Success Criteria
- ✅ KML file is read and boundary extracted
- ✅ Boundary data is compressed (should be ~200 bytes for typical polygon)
- ✅ Data transmitted in chunks over 3DR radio
- ✅ Drone receives, decompresses, and generates waypoints
- ✅ Mission uploaded to Pixhawk successfully
- ✅ User can ARM → TAKEOFF → MODE:AUTO to start mission
- ✅ All existing commands (PING, ARM, TAKEOFF, etc.) still work

---

**Implementation Task**: Update `tx_commands.py` to add KML boundary transmission as described above. Keep all existing functionality working.
