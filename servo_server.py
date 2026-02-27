#!/usr/bin/env python3
"""
Servo Controller Server for Raspberry Pi 4
Receives pose data and controller inputs from Pi 5 via TCP socket.
Controls camera gimbal servos and vehicle functions.
"""

import asyncio
import json
import time

try:
    import websockets
    WEBSOCKETS_AVAILABLE = True
except ImportError:
    print("websockets library not found. WebSocket test UI will be unavailable.")
    print("Install with: pip install websockets")
    WEBSOCKETS_AVAILABLE = False

HOST = '0.0.0.0'  # Listen on all interfaces
PORT = 9090       # TCP servo control port (Pi 5)
WS_PORT = 8080    # WebSocket port (test UI)

# For PCA9685 servo control using local library
try:
    import PCA9685
    HARDWARE_AVAILABLE = True
except ImportError:
    print("PCA9685 library not found. Servo control disabled (simulation mode).")
    HARDWARE_AVAILABLE = False


class ServoController:
    """Manages all servo and PWM outputs for VR headset robot control"""
    
    # Servo channel assignments
    CAMERA_TILT = 0   # X-axis (pitch)
    CAMERA_PAN = 1    # Y-axis (yaw)
    BLOWER_TILT = 2   # Blower tilt servo
    BLOWER_PAN = 3    # Blower pan servo
    
    # ESC channel assignments (tracked chassis)
    ESC_FORWARD_BACK = 4  # Forward/back ESC (1500 neutral, >1500 forward, <1500 back)
    ESC_LEFT_RIGHT = 5    # Left/right ESC (1500 neutral, >1500 right, <1500 left)
    ESC_JET = 8
    # PWM channel assignments
    LIGHTS = 6        # LED lights PWM
    HORN = 7          # Horn PWM
    
    def __init__(self):
        """Initialize PCA9685 and set default positions"""
        if HARDWARE_AVAILABLE:
            self.pca = PCA9685.PCA9685(address=0x40, debug=False)
            self.pca.setPWMFreq(50)  # Standard servo frequency
            print("✓ PCA9685 initialized at 50Hz")
        else:
            self.pca = None
            print("⚠ Running in simulation mode (no hardware)")
        
        # Current servo positions (in degrees)
        self.camera_tilt = 135  # Center position
        self.camera_pan = 135   # Center position
        self.blower_tilt = 135  # Center position
        self.blower_pan = 135   # Center position
        
        # Blower joystick rate-based control speed (degrees per update)
        self.blower_speed = 3.0
        
        # ESC positions (1000-2000 µs, 1500 neutral)
        self.esc_forward_back = 1500
        self.esc_left_right = 1500
        self.esc_jet = 2200
        
        # PWM states (0-255)
        self.lights = 0
        self.horn = 0
        
        # Set initial positions
        self.reset_all()
        self.spin_up_jet()
    
    def spin_up_jet(self):
        """Turn on jet blower"""
        time.sleep(1)
        self.set_esc(self.ESC_JET, 1500)

        time.sleep(2)

        self.set_esc(self.ESC_JET, 2500)
    
    def angle_to_pulse(self, angle, min_pulse=500, max_pulse=2500):
        """Convert angle (0-270°) to PWM pulse (microseconds)"""
        pulse = min_pulse + (angle / 270.0) * (max_pulse - min_pulse)
        return int(pulse)
    
    def set_servo(self, channel, angle):
        """Set servo position by angle (0-270°)"""
        pulse = self.angle_to_pulse(angle)
        if self.pca:
            self.pca.setServoPulse(channel, pulse)
        else:
            print(f"[SIM] Servo {channel} → {angle}° (pulse: {pulse}µs)")
    
    def set_pwm(self, channel, value):
        """Set PWM output (0-255)"""
        # Convert 0-255 to 0-4095 (12-bit PWM)
        pwm_value = int((value / 255.0) * 4095)
        if self.pca:
            self.pca.setServoPulse(channel, pwm_value)
        else:
            print(f"[SIM] PWM {channel} → {value}/255 ({pwm_value}/4095)")
    
    def set_esc(self, channel, pulse):
        """Set ESC pulse directly (1000-2000 µs, 1500 neutral)"""
        # Clamp to safe range
        pulse = max(1000, min(2000, pulse))
        if self.pca:
            self.pca.setServoPulse(channel, pulse)
        else:
            print(f"[SIM] ESC {channel} → {pulse}µs")
    
    def update_camera_from_quaternion(self, quat):
        """Update camera gimbal from headset quaternion (x, y, z, w)"""
        # Convert quaternion to Euler angles
        # For now, simple mapping from quaternion components
        x, y, z, w = quat.get('x', 0), quat.get('y', 0), quat.get('z', 0), quat.get('w', 1)
        
        # Map quaternion X (head tilt) to camera tilt servo
        # Range: [-1, 1] → [0, 270°]
        self.camera_tilt = (x + 1) * 135
        self.set_servo(self.CAMERA_TILT, self.camera_tilt)
        
        # Map quaternion Y (head pan) to camera pan servo
        self.camera_pan = (y + 1) * 135
        self.set_servo(self.CAMERA_PAN, self.camera_pan)
    
    def update_esc_from_joystick(self, joy_x, joy_y):
        """
        Update ESC pulses from joystick inputs (right controller)
        joy_x: left/right (-1 to 1) → ESC_LEFT_RIGHT (1000-2000)
        joy_y: forward/back (-1 to 1) → ESC_FORWARD_BACK (1000-2000)
        """
        # Convert joystick values (-1 to 1) to ESC pulse (1000-2000, neutral 1500)
        # Y-axis: -1 (back) to 1 (forward)
        self.esc_forward_back = int(1500 + (joy_y * 500))
        # X-axis: -1 (left) to 1 (right)  
        self.esc_left_right = int(1500 + (joy_x * 500))
        
        # Send to ESCs
        self.set_esc(self.ESC_FORWARD_BACK, self.esc_forward_back)
        self.set_esc(self.ESC_LEFT_RIGHT, self.esc_left_right)
        
        return self.esc_forward_back, self.esc_left_right
    
    def update_blower_from_joystick(self, joy_x, joy_y):
        """
        Update blower pan/tilt from left joystick using rate-based control.
        joy_x: left/right (-1 to 1) → pan (negative = toward 270°/left, positive = toward 0°/right)
        joy_y: up/down (-1 to 1) → tilt (positive = toward 0°/down, negative = toward 270°/up)
        Joystick at center (0) = hold current position.
        """
        # Rate-based: joystick deflection controls speed, not position
        pan_delta = -joy_x * self.blower_speed
        tilt_delta = -joy_y * self.blower_speed
        
        # Apply deltas and clamp to servo range
        self.blower_pan = max(0, min(270, self.blower_pan + pan_delta))
        self.blower_tilt = max(0, min(270, self.blower_tilt + tilt_delta))
        
        self.set_servo(self.BLOWER_PAN, self.blower_pan)
        self.set_servo(self.BLOWER_TILT, self.blower_tilt)
        
        return self.blower_pan, self.blower_tilt
    
    def set_lights(self, brightness):
        """Set LED lights brightness (0-255)"""
        self.lights = brightness
        self.set_pwm(self.LIGHTS, brightness)
    
    def trigger_horn(self, duration=0.2):
        """Trigger horn for short duration"""
        self.horn = 255
        self.set_pwm(self.HORN, 255)
        # Note: You'll need to turn it off after duration (use asyncio task)
    
    def reset_all(self):
        """Reset all servos to center/safe positions"""
        self.set_servo(self.CAMERA_TILT, 135)
        self.set_servo(self.CAMERA_PAN, 135)
        self.set_servo(self.BLOWER_TILT, 135)
        self.set_servo(self.BLOWER_PAN, 135)
        self.set_esc(self.ESC_FORWARD_BACK, 1500)  # Neutral
        self.set_esc(self.ESC_LEFT_RIGHT, 1500)    # Neutral
        self.set_esc(self.ESC_JET, 500)
        self.set_pwm(self.LIGHTS, 0)
        self.set_pwm(self.HORN, 0)
        print("✓ All servos and ESCs reset to neutral positions")


# Global servo controller instance
servo_controller = ServoController()

async def process_message(msg):
    """Process a parsed JSON message (shared by TCP and WebSocket handlers)"""
    msg_type = msg.get('type', 'unknown')
    
    if msg_type == 'pose':
        # Head tracking data
        orient = msg.get('orientation', {})
        servo_controller.update_camera_from_quaternion(orient)
    
    elif msg_type == 'controller':
        # Joystick and button inputs
        left_joy = msg.get('leftJoystick', {})
        right_joy = msg.get('rightJoystick', {})
        buttons = msg.get('buttons', {})
        
        # Update blower pan/tilt from left joystick
        blower_pan, blower_tilt = servo_controller.update_blower_from_joystick(
            left_joy.get('x', 0),
            left_joy.get('y', 0)
        )
        
        # Update ESCs from right joystick
        esc_fb, esc_lr = servo_controller.update_esc_from_joystick(
            right_joy.get('x', 0),
            right_joy.get('y', 0)
        )
        
        # Handle buttons
        if buttons.get('lights'):
            print("lights toggle pressed")
            new_brightness = 255 if servo_controller.lights == 0 else 0
            servo_controller.set_lights(new_brightness)
        
        if buttons.get('horn'):
            print("horn pressed")
            servo_controller.trigger_horn()
        
        print(f"[CTRL] Joy L:({left_joy.get('x', 0):.2f},{left_joy.get('y', 0):.2f}) "
              f"Blower P:{blower_pan:.1f}° T:{blower_tilt:.1f}° | "
              f"Joy R:({right_joy.get('x', 0):.2f},{right_joy.get('y', 0):.2f}) "
              f"ESC FB:{esc_fb}µs LR:{esc_lr}µs")
    
    else:
        print(f"Unknown message type: {msg_type}")


async def handle_client(reader, writer):
    addr = writer.get_extra_info('peername')
    print(f"Pi 5 connected from {addr}")
    
    try:
        while True:
            # Read until newline delimiter
            data = await reader.readline()
            if not data:
                print(f"Pi 5 disconnected from {addr}")
                break
            
            try:
                # Parse JSON message
                message = data.decode('utf-8').strip()
                msg = json.loads(message)
                await process_message(msg)
                
            except json.JSONDecodeError as e:
                print(f"Invalid JSON received: {e}")
            except Exception as e:
                print(f"Error processing message: {e}")
                import traceback
                traceback.print_exc()
                
    except asyncio.CancelledError:
        print(f"Connection handler cancelled for {addr}")
    except Exception as e:
        print(f"Connection error: {e}")
    finally:
        writer.close()
        await writer.wait_closed()
        print(f"Connection closed for {addr}")

async def handle_websocket(websocket):
    """Handle WebSocket connections from test UI"""
    addr = websocket.remote_address
    print(f"WebSocket client connected from {addr}")
    
    try:
        async for raw_message in websocket:
            try:
                msg = json.loads(raw_message)
                await process_message(msg)
                await websocket.send(json.dumps({"status": "ok"}))
            except json.JSONDecodeError as e:
                print(f"Invalid JSON from WebSocket: {e}")
                await websocket.send(json.dumps({"status": "error", "message": str(e)}))
            except Exception as e:
                print(f"Error processing WebSocket message: {e}")
                await websocket.send(json.dumps({"status": "error", "message": str(e)}))
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        print(f"WebSocket client disconnected from {addr}")


async def main():
    # Start TCP server for Pi 5
    tcp_server = await asyncio.start_server(
        handle_client, HOST, PORT
    )
    
    addrs = ', '.join(str(sock.getsockname()) for sock in tcp_server.sockets)
    print(f"TCP servo server listening on {addrs}")
    print("Waiting for Pi 5 connection...")
    
    # Start WebSocket server for test UI
    if WEBSOCKETS_AVAILABLE:
        ws_server = await websockets.serve(handle_websocket, HOST, WS_PORT)
        print(f"WebSocket test server listening on {HOST}:{WS_PORT}")
        print(f"Open servo_test.html in a browser to control")
        
        async with tcp_server:
            await asyncio.gather(
                tcp_server.serve_forever(),
                ws_server.wait_closed()
            )
    else:
        async with tcp_server:
            await tcp_server.serve_forever()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nServo server stopped")