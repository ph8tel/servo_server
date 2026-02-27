# Servo Server for PCA9685-controlled robot

This repository runs a small servo/ESC server for a Raspberry Pi that controls a camera gimbal, chassis ESCs, and a blower mounted on a pan/tilt pair driven by a PCA9685 board.

Key points
- TCP server on port 9090: primary control channel used by the Pi 5 (JSON messages, newline-delimited).
- WebSocket server on port 8080: test UI (`servo_test.html`) connects here so browsers can control the robot in real time.
- Simulation mode: if the `PCA9685` library is unavailable the server prints simulated actions instead of talking to hardware.

Message protocol
- `pose` messages: used for camera gimbal updates.
	- Example: {"type":"pose","orientation":{"x":0.0,"y":0.0,"z":0.0,"w":1.0}}
- `controller` messages: used for joysticks and buttons.
	- Example: {"type":"controller","leftJoystick":{"x":-0.2,"y":0.4},"rightJoystick":{"x":0.0,"y":0.0},"buttons":{}}
	- `leftJoystick` controls the blower pan/tilt (rate-based).
	- `rightJoystick` controls tracked-chassis ESCs (forward/back and left/right).

Hardware channel mapping (PCA9685 channels)
- Channel 0: Camera tilt (0–270°)
- Channel 1: Camera pan (0–270°)
- Channel 2: Blower tilt (0–270°)
- Channel 3: Blower pan (0–270°)
- Channel 4: ESC forward/back (1000–2000 µs)
- Channel 5: ESC left/right (1000–2000 µs)
- Channel 6: Lights (PWM 0–255)
- Channel 7: Horn (PWM trigger)
- Channel 8: ESC jet blower

Blower control behavior
- The blower pan/tilt is driven from `leftJoystick` using rate-based control.
	- Negative `leftJoystick.x` moves pan toward 270° (left limit), positive toward 0° (right).
	- Positive `leftJoystick.y` tilts down (toward 0°), negative tilts up (toward 270°).
	- Joystick centered = hold current position.

Running the server (development)
1. Install optional WebSocket dependency for the test UI:

	 pip install websockets

2. Run the server (TCP + WebSocket):

	 python3 New/servo_server.py

3. Open `servo_test.html` in a browser and click "Connect" — the test UI will connect to the WebSocket on port 8080.

Notes
- The TCP socket on port 9090 remains the primary interface for Pi-to-Pi communications and is unchanged.
- `servo_test.html` now uses a native WebSocket to send the same JSON messages the server expects.
- If you run without the `PCA9685` library the server will print simulated servo/ESC actions for testing.