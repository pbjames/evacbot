from controller import Robot, Motor, Keyboard, Camera, GPS, InertialUnit, Gyro, Compass, LED
import math
import requests
import numpy as np
import cv2
import threading

def clamp(value, low, high):
    return max(low, min(high, value))

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# --- Devices ---
camera = robot.getDevice("camera")
camera.enable(timestep)

imu = robot.getDevice("inertial unit")
imu.enable(timestep)

gps = robot.getDevice("gps")
gps.enable(timestep)

gyro = robot.getDevice("gyro")
gyro.enable(timestep)

compass = robot.getDevice("compass")
compass.enable(timestep)

keyboard = robot.getKeyboard()
keyboard.enable(timestep)

front_left_led = robot.getDevice("front left led")
front_right_led = robot.getDevice("front right led")

camera_roll = robot.getDevice("camera roll")
camera_pitch = robot.getDevice("camera pitch")

# Motors
m_fl = robot.getDevice("front left propeller")
m_fr = robot.getDevice("front right propeller")
m_rl = robot.getDevice("rear left propeller")
m_rr = robot.getDevice("rear right propeller")

motors = [m_fl, m_fr, m_rl, m_rr]
for m in motors:
    m.setPosition(float("inf"))
    m.setVelocity(1.0)

print("Start the drone...")

# PID constants (same as C)
k_vertical_thrust = 68.5
k_vertical_offset = 0.6
k_vertical_p = 3.0
k_roll_p = 50.0
k_pitch_p = 30.0

target_altitude = 1.0
detection_enabled = False
frame_counter = 0
SEND_EVERY_N_FRAMES = 5

print("Controls:")
print("Arrow keys: movement")
print("Shift + arrows: altitude / strafing")
print("K = enable detection, L = disable detection")

# limit concurrent sends
send_lock = threading.Lock()

# ---------- Async frame sender ----------
def send_frame_async(jpeg_bytes: bytes):
    """
    Send JPEG to YOLO server but never allow more than 1 in-flight request.
    This prevents thread buildup and freezing.
    """
    if not send_lock.acquire(blocking=False):
        return  # previous send still running → skip this frame

    def worker(data):
        try:
            requests.post(
                "http://127.0.0.1:5001/frame",
                files={"file": ("frame.jpg", data, "image/jpeg")},
                timeout=1,
            )
        except Exception as e:
            print("POST error:", e)
        finally:
            send_lock.release()

    threading.Thread(target=worker, args=(jpeg_bytes,), daemon=True).start()

# --- Main Loop ---
while robot.step(timestep) != -1:

    time_now = robot.getTime()

    # Sensors
    roll, pitch, yaw = imu.getRollPitchYaw()
    altitude = gps.getValues()[2]
    roll_velocity = gyro.getValues()[0]
    pitch_velocity = gyro.getValues()[1]

    # LED blinking
    led_state = int(time_now) % 2
    front_left_led.set(led_state)
    front_right_led.set(1 - led_state)

    # Camera gimbal stabilization
    camera_roll.setPosition(-0.115 * roll_velocity)
    camera_pitch.setPosition(-0.1 * pitch_velocity)

    # Keyboard control
    roll_dist = 0
    pitch_dist = 0
    yaw_dist = 0

    key = keyboard.getKey()
    while key != -1:
        if key == Keyboard.UP:
            pitch_dist = -2
        elif key == Keyboard.DOWN:
            pitch_dist = 2
        elif key == Keyboard.RIGHT:
            yaw_dist = -1.3
        elif key == Keyboard.LEFT:
            yaw_dist = 1.3
        elif key == (Keyboard.SHIFT + Keyboard.RIGHT):
            roll_dist = -1
        elif key == (Keyboard.SHIFT + Keyboard.LEFT):
            roll_dist = 1
        elif key == (Keyboard.SHIFT + Keyboard.UP):
            target_altitude += 0.05
            print("target altitude:", target_altitude)
        elif key == (Keyboard.SHIFT + Keyboard.DOWN):
            target_altitude -= 0.05
            print("target altitude:", target_altitude)

        elif key == ord('K') or key == ord('k'):
            detection_enabled = True
            print("Hazard detection ENABLED.")

        elif key == ord('L') or key == ord('l'):
            detection_enabled = False
            print("Hazard detection DISABLED.")

        key = keyboard.getKey()

    # --- PID control calculations ---
    roll_input = k_roll_p * clamp(roll, -1, 1) + roll_velocity + roll_dist
    pitch_input = k_pitch_p * clamp(pitch, -1, 1) + pitch_velocity + pitch_dist
    yaw_input = yaw_dist

    diff_alt = clamp(target_altitude - altitude + k_vertical_offset, -1, 1)
    vertical_input = k_vertical_p * (diff_alt ** 3)

    # Motor power calculation
    fl = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input
    fr = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input
    rl = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input
    rr = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input

    # --- Non-blocking detection: send frame to YOLO server ---
    if detection_enabled:
        frame_counter += 1
        if frame_counter % SEND_EVERY_N_FRAMES == 0:
            raw = camera.getImage()
            if raw is not None:
                width = camera.getWidth()
                height = camera.getHeight()

                # Webots image is BGRA (4 channels)
                img = np.frombuffer(raw, np.uint8).reshape((height, width, 4))
                bgr = img[:, :, :3]

                ok, jpeg = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
                if ok:
                    send_frame_async(jpeg.tobytes())

    # Apply motor commands
    m_fl.setVelocity(fl)
    m_fr.setVelocity(-fr)
    m_rl.setVelocity(-rl)
    m_rr.setVelocity(rr)
