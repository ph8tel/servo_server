#!/usr/bin/env python3
"""
Servo Controller Server for Raspberry Pi 4
Receives pose data and controller inputs from Pi 5 via TCP socket.
Controls camera gimbal servos and vehicle functions.
"""

import asyncio
import json
import time
import math

try:
    import websockets
    WEBSOCKETS_AVAILABLE = True
except ImportError:
    print("websockets library not found. WebSocket test UI will be unavailable.")
    print("Install with: pip install websockets")
    WEBSOCKETS_AVAILABLE = False

HOST = '0.0.0.0'
PORT = 9090
WS_PORT = 8080

try:
    import PCA9685
    HARDWARE_AVAILABLE = True
except ImportError:
    print("PCA9685 library not found. Servo control disabled (simulation mode).")
    HARDWARE_AVAILABLE = False


# ------------------------------------------------------------
#  ESC FILTER (accel/decel smoothing)
# ------------------------------------------------------------
class EscFilter:
    """
    Smooths ESC commands using asymmetric slew rate limiting.
    accel_rate: max PWM change per second when accelerating away from 1500
    decel_rate: max PWM change per second when returning toward 1500
    """

    def __init__(self, accel_rate=300, decel_rate=900):
        self.accel_rate = accel_rate
        self.decel_rate = decel_rate
        self.filtered = 1500
        self.last_time = time.time()

    def update(self, target):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        cur = self.filtered
        neutral = 1500

        accelerating = (
            (target > cur and cur >= neutral) or
            (target < cur and cur <= neutral)
        )

        rate = self.accel_rate if accelerating else self.decel_rate
        max_step = rate * dt

        delta = target - cur

        if abs(delta) <= max_step:
            self.filtered = target
        else:
            self.filtered = cur + math.copysign(max_step, delta)

        return int(self.filtered)


# ------------------------------------------------------------
#  MAIN SERVO CONTROLLER
# ------------------------------------------------------------
class ServoController:
    CAMERA_TILT = 0
    CAMERA_PAN = 1
    BLOWER_TILT = 2
    BLOWER_PAN = 3

    ESC_FORWARD_BACK = 4
    ESC_LEFT_RIGHT = 5
    ESC_JET = 8

    LIGHTS = 6
    HORN = 7

    def __init__(self):
        if HARDWARE_AVAILABLE:
            self.pca = PCA9685.PCA9685(address=0x40, debug=False)
            self.pca.setPWMFreq(50)
            print("✓ PCA9685 initialized at 50Hz")
        else:
            self.pca = None
            print("⚠ Running in simulation mode (no hardware)")

        self.camera_tilt = 135
        self.camera_pan = 135
        self.blower_tilt = 135
        self.blower_pan = 135

        self.blower_speed = 3.0

        self.esc_forward_back = 1500
        self.esc_left_right = 1500
        self.esc_jet = 0

        self.lights = 0
        self.horn = 0

        # ------------------------------------------------------------
        #  ESC FILTERS 
        # ------------------------------------------------------------
        self.fb_filter = EscFilter(accel_rate=300, decel_rate=900)
        self.lr_filter = EscFilter(accel_rate=300, decel_rate=900)
        
        self.jet_filter = EscFilter(accel_rate=400, decel_rate=600)

        self.reset_all()
        self.spin_up_jet()
    
    def update_jet(self, target_pwm):
        smooth = self.jet_filter.update(target_pwm)
        self.esc_jet = smooth
        self.set_esc(self.ESC_JET, smooth)
        return smooth

    def spin_up_jet(self):
        # Send zero-throttle to arm the ESC
        self.set_esc(self.ESC_JET, 1000)
        time.sleep(1.0) 

        #init filtered as zero-throttle
        self.jet_filter.filtered = 1000 
        self.esc_jet = 1500
    
    def jet_75(self): 
        return self.update_jet(1750) 
    
    def jet_100(self): 
        return self.update_jet(2000)
    
    def jet_off(self):
        return self.update_jet(1000)


    def angle_to_pulse(self, angle, min_pulse=500, max_pulse=2500):
        pulse = min_pulse + (angle / 270.0) * (max_pulse - min_pulse)
        return int(pulse)

    def set_servo(self, channel, angle):
        pulse = self.angle_to_pulse(angle)
        if self.pca:
            self.pca.setServoPulse(channel, pulse)
        else:
            print(f"[SIM] Servo {channel} → {angle}° (pulse: {pulse}µs)")

    def set_pwm(self, channel, value):
        pwm_value = int((value / 255.0) * 4095)
        if self.pca:
            self.pca.setServoPulse(channel, pwm_value)
        else:
            print(f"[SIM] PWM {channel} → {value}/255 ({pwm_value}/4095)")

    def set_esc(self, channel, pulse):
        pulse = max(1000, min(2000, pulse))
        if self.pca:
            self.pca.setServoPulse(channel, pulse)
        else:
            print(f"[SIM] ESC {channel} → {pulse}µs")

    def update_camera_from_quaternion(self, quat):
        x, y, z, w = quat.get('x', 0), quat.get('y', 0), quat.get('z', 0), quat.get('w', 1)
        self.camera_tilt = (x + 1) * 135
        self.camera_pan = (y + 1) * 135
        self.set_servo(self.CAMERA_TILT, self.camera_tilt)
        self.set_servo(self.CAMERA_PAN, self.camera_pan)

    # ------------------------------------------------------------
    #  UPDATED ESC LOGIC WITH SMOOTHING
    # ------------------------------------------------------------
    def update_esc_from_joystick(self, joy_x, joy_y):
        """
        joy_x: left/right (-1 to 1)
        joy_y: forward/back (-1 to 1)
        """

        # Optional joystick curve (disabled by default)
        # joy_x = math.copysign(abs(joy_x)**1.8, joy_x)
        # joy_y = math.copysign(abs(joy_y)**1.8, joy_y)

        raw_fb = int(1500 + (joy_y * 500))
        raw_lr = int(1500 + (joy_x * 500))

        # Apply smoothing
        smooth_fb = self.fb_filter.update(raw_fb)
        smooth_lr = self.lr_filter.update(raw_lr)

        self.esc_forward_back = smooth_fb
        self.esc_left_right = smooth_lr

        self.set_esc(self.ESC_FORWARD_BACK, smooth_fb)
        self.set_esc(self.ESC_LEFT_RIGHT, smooth_lr)

        return smooth_fb, smooth_lr

    def update_blower_from_joystick(self, joy_x, joy_y):
        pan_delta = -joy_x * self.blower_speed
        tilt_delta = -joy_y * self.blower_speed

        self.blower_pan = max(0, min(270, self.blower_pan + pan_delta))
        self.blower_tilt = max(0, min(270, self.blower_tilt + tilt_delta))

        self.set_servo(self.BLOWER_PAN, self.blower_pan)
        self.set_servo(self.BLOWER_TILT, self.blower_tilt)

        return self.blower_pan, self.blower_tilt

    def set_lights(self, brightness):
        self.lights = brightness
        self.set_pwm(self.LIGHTS, brightness)

    def trigger_horn(self, duration=0.2):
        self.horn = 255
        self.set_pwm(self.HORN, 255)

    def reset_all(self):
        self.set_servo(self.CAMERA_TILT, 135)
        self.set_servo(self.CAMERA_PAN, 135)
        self.set_servo(self.BLOWER_TILT, 135)
        self.set_servo(self.BLOWER_PAN, 135)
        self.set_esc(self.ESC_FORWARD_BACK, 1500)
        self.set_esc(self.ESC_LEFT_RIGHT, 1500)
        self.set_esc(self.ESC_JET, 500)
        self.set_pwm(self.LIGHTS, 0)
        self.set_pwm(self.HORN, 0)
        print("✓ All servos and ESCs reset to neutral positions")


servo_controller = ServoController()


async def process_message(msg):
    msg_type = msg.get('type', 'unknown')

    if msg_type == 'pose':
        orient = msg.get('orientation', {})
        servo_controller.update_camera_from_quaternion(orient)

    elif msg_type == 'controller':
        left_joy = msg.get('leftJoystick', {})
        right_joy = msg.get('rightJoystick', {})
        buttons = msg.get('buttons', {})

        blower_pan, blower_tilt = servo_controller.update_blower_from_joystick(
            left_joy.get('x', 0),
            left_joy.get('y', 0)
        )

        esc_fb, esc_lr = servo_controller.update_esc_from_joystick(
            right_joy.get('x', 0),
            right_joy.get('y', 0)
        )

        if buttons.get('lights'):
            print("lights toggle pressed")
            new_brightness = 255 if servo_controller.lights == 0 else 0
            servo_controller.set_lights(new_brightness)

        if buttons.get('horn'):
            print("horn pressed")
            servo_controller.trigger_horn()
        
        if buttons.get('fan75'): 
            servo_controller.jet_75() 

        if buttons.get('fan100'): 
            servo_controller.jet_100() 
        
        if buttons.get('fanOff'): 
            servo_controller.jet_off()

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
            data = await reader.readline()
            if not data:
                print(f"Pi 5 disconnected from {addr}")
                break

            try:
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
    tcp_server = await asyncio.start_server(handle_client, HOST, PORT)

    addrs = ', '.join(str(sock.getsockname()) for sock in tcp_server.sockets)
    print(f"TCP servo server listening on {addrs}")
    print("Waiting for Pi 5 connection...")

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
