from curses import KEY_UP
from typing import Final
from controller.motor import Motor
from controller.robot import Robot
import math

# Empirically found constants ripped from the source controller
K_VERTICAL_THRUST: Final[float] = 68.5
# with this thrust, the drone lifts.
K_VERTICAL_OFFSET: Final[float] = 0.6
# Vertical offset where the robot actually targets to stabilize itself.
K_VERTICAL_P: Final[float] = 3.0
# P constant of the vertical PID.
K_ROLL_P: Final[float] = 50.0
# P constant of the roll PID.
K_PITCH_P: Final[float] = 30.0
# P constant of the pitch PID.
INITIAL_VELOCITY: Final[float] = 1.0
robot = Robot()
timestep = int(robot.getBasicTimeStep())
camera = robot.getCamera("camera")
camera.enable(timestep)
front_left_led = robot.getLED("front left led")
front_right_led = robot.getLED("front right led")
imu = robot.getInertialUnit("inertial unit")
imu.enable(timestep)
gps = robot.getGPS("gps")
gps.enable(timestep)
compass = robot.getCompass("compass")
compass.enable(timestep)
gyro = robot.getGyro("gyro")
gyro.enable(timestep)
camera_roll_motor = robot.getMotor("camera roll motor")
camera_pitch_motor = robot.getMotor("camera pitch motor")
# camera_yaw_motor = robot.getMotor("camera yaw motor")
front_left_motor = robot.getMotor("front left propeller")
front_right_motor = robot.getMotor("front right propeller")
rear_left_motor = robot.getMotor("rear left propeller")
rear_right_motor = robot.getMotor("rear right propeller")
motors: list[Motor] = [
    front_left_motor,
    front_right_motor,
    rear_left_motor,
    rear_right_motor,
]
for motor in motors:
    motor.setPosition(math.inf)
    motor.setVelocity(INITIAL_VELOCITY)
print("Starting the drone..")
while robot.step(timestep) != -1:
    if robot.getTime() > 1.0:
        break
print("You can control the drone with your computer keyboard:")
print("- 'up': move forward.")
print("- 'down': move backward.")
print("- 'right': turn right.")
print("- 'left': turn left.")
print("- 'shift + up': increase the target altitude.")
print("- 'shift + down': decrease the target altitude.")
print("- 'shift + right': strafe right.")
print("- 'shift + left': strafe left.")
print("You can control the drone with your computer keyboard:")
print("- 'up': move forward.")
print("- 'down': move backward.")
print("- 'right': turn right.")
print("- 'left': turn left.")
print("- 'shift + up': increase the target altitude.")
print("- 'shift + down': decrease the target altitude.")
print("- 'shift + right': strafe right.")
print("- 'shift + left': strafe left.")
target_altitude = 1
while robot.step(timestep) != -1:
    time = robot.getTime()
    roll, pitch, yaw = imu.getRollPitchYaw()
    altitude = gps.getValues()[2]
    roll_velocity, pitch_velocity = gyro.getValues()
    if int(time) % 2:
        left_led_value = front_left_led.value
        front_left_led.set(not left_led_value)
        front_right_led.set(left_led_value)
    camera_roll_motor.setPosition(-0.115 * roll_velocity)
    camera_pitch_motor.setPosition(-0.1 * pitch_velocity)
    roll_disturbance = pitch_disturbance = yaw_disturbance = 0.0
    key = robot.getKeyboard().getKey()
    while (key > 0):
        if key == KEY_UP:
            pitch_disturbance = -2
        elif key == KEY_UP:
            pitch_disturbance = 2
# TODO: Finish ripping from the source
