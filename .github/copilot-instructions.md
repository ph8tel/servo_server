# Servo Server Copilot Instructions

## Project Overview

Servo control server for a two-Pi system: a Pi 5 (VR/headset/controller input) sends JSON messages to a Pi 4 which drives a PCA9685 board to control servos and ESCs.

## Architecture

**Two-Pi System:**
- **Pi 5** (not in this repo): sends pose + controller data via TCP to the Pi 4
- **Pi 4** (this repo): `New/servo_server.py` accepts TCP from Pi 5 (port 9090) and also exposes a WebSocket test server (port 8080) for browser-based testing

**Communication Protocol:**
- TCP socket server on port 9090 (newline-delimited JSON)
- WebSocket server on port 8080 for `servo_test.html` (browser test UI)
- JSON messages used by both transports with the same `type` field

**Message Types:**
- `pose`: Headset quaternion → camera gimbal servos
- `controller`: Joysticks + buttons → blower pan/tilt + ESCs + accessories

## Hardware Mappings (PCA9685 Channels)

Current critical assignments in `ServoController`:
```python
CAMERA_TILT = 0      # 0-270°
CAMERA_PAN = 1       # 0-270°
BLOWER_TILT = 2      # 0-270° (left joystick Y controls tilt rate)
BLOWER_PAN = 3       # 0-270° (left joystick X controls pan rate)
ESC_FORWARD_BACK = 4 # 1000-2000 µs
ESC_LEFT_RIGHT = 5   # 1000-2000 µs
LIGHTS = 6           # PWM 0-255
HORN = 7             # PWM trigger
ESC_JET = 8          # Jet blower channel
```

## Key Patterns

### Message Protocol
Messages are simple JSON objects. Examples:
```json
{"type":"pose","orientation":{"x":0.1,"y":-0.2,"z":0.0,"w":1.0}}
{"type":"controller","leftJoystick":{"x":-0.5,"y":0.2},"rightJoystick":{"x":0.0,"y":0.0},"buttons":{}}
```

### Transforms
- `pose` → `update_camera_from_quaternion()` (simple linear mapping of `x`/`y` to tilt/pan)
- `controller` →
  - `leftJoystick` (x/y) drives blower pan/tilt via `update_blower_from_joystick()` (rate-based)
  - `rightJoystick` (x/y) drives ESC pulses via `update_esc_from_joystick()`

### Simulation Mode
If `PCA9685` is not installed the server prints simulated actions. WebSocket support requires the `websockets` package; if it's not present the TCP server still runs normally.

## Development Workflows

### Running the Server
Install optional dependency for the browser test UI:

```bash
pip install websockets
```

Run the server (starts both TCP and WebSocket listeners):

```bash
python3 New/servo_server.py
```

### Testing Without Hardware
1. Start the server in simulation mode (no `PCA9685` installed)
2. Open `servo_test.html` in a browser
3. Click Connect (the page opens a WebSocket to port 8080). The page now sends the same JSON messages the server expects.

## Adding New Hardware
1. Assign a channel constant in `ServoController` and update `reset_all()`
2. Add a control method (use `set_servo()` for 0-270°, `set_esc()` for 1000-2000 µs)
3. Wire it into `process_message()` so both TCP and WebSocket clients benefit

## Critical Details
- **ESC Safety:** ESCs default to 1500 µs on startup
- **Blower pan/tilt:** rate-based movement from `leftJoystick`
- **Jet blower:** `spin_up_jet()` handles initial spin behavior on startup (channel 8)
- **Async I/O:** `asyncio` powers TCP and WebSocket handlers
- **Error Handling:** JSON parse errors are logged but connections remain alive

## File Map
- `New/servo_server.py`: Main server (TCP + WebSocket) and PCA9685 control logic
- `servo_test.html`: Browser-based test UI that connects via WebSocket to port 8080
- `README.md`: Project overview and run instructions

When making changes keep simulation fallback and message formats consistent for both TCP and WebSocket clients.
