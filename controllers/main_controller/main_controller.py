from typing import Final
from controller.motor import Motor
from controller.robot import Robot
import math

# Empirically found constants ripped from the source controller
K_VERTICAL_THRUST: Final[float] = 68.5  # with this thrust, the drone lifts.
K_VERTICAL_OFFSET: Final[float] = (
    0.6  # Vertical offset where the robot actually targets to stabilize itself.)
)
K_VERTICAL_P: Final[float] = 3.0  # P constant of the vertical PID.
K_ROLL_P: Final[float] = 50.0  # P constant of the roll PID.
K_PITCH_P: Final[float] = 30.0  # P constant of the pitch PID.
INITIAL_VELOCITY: Final[float] = 1.0


def produce_motor_velocities(
    roll: float,
    pitch: float,
    roll_velocity: float,
    pitch_velocity: float,
    roll_disturbance: float,
    pitch_disturbance: float,
    yaw_disturbance: float,
    altitude: float,
    target_altitude: float,
) -> tuple[float, float, float, float]:
    def clamp(value: float, min_v: float, max_v: float):
        return min(max_v, max(value, min_v))

    roll_input = K_ROLL_P * clamp(roll, -1.0, 1.0) + roll_velocity + roll_disturbance
    pitch_input = (
        K_PITCH_P * clamp(pitch, -1.0, 1.0) + pitch_velocity + pitch_disturbance
    )
    yaw_input = yaw_disturbance
    clamped_difference_altitude = clamp(
        target_altitude - altitude + K_VERTICAL_OFFSET, -1.0, 1.0
    )
    vertical_input = K_VERTICAL_P * (clamped_difference_altitude**3)
    return (
        K_VERTICAL_THRUST + vertical_input - roll_input + pitch_input - yaw_input,
        K_VERTICAL_THRUST + vertical_input + roll_input + pitch_input + yaw_input,
        K_VERTICAL_THRUST + vertical_input - roll_input - pitch_input + yaw_input,
        K_VERTICAL_THRUST + vertical_input + roll_input - pitch_input - yaw_input,
    )


def main():
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
        roll, pitch, _ = imu.getRollPitchYaw()
        altitude = gps.getValues()[2]
        roll_velocity, pitch_velocity = gyro.getValues()
        if int(time) % 2:
            left_led_value = front_left_led.value
            front_left_led.set(not left_led_value)
            front_right_led.set(left_led_value)
        camera_roll_motor.setPosition(-0.115 * roll_velocity)
        camera_pitch_motor.setPosition(-0.1 * pitch_velocity)
        roll_disturbance = pitch_disturbance = yaw_disturbance = 0.0
        while (key := robot.getKeyboard().getKey()) > 0:
            if key == ord("k"):
                pitch_disturbance = -2.0
            elif key == ord("j"):
                pitch_disturbance = 2.0
            elif key == ord("l"):
                yaw_disturbance = -1.3
            elif key == ord("h"):
                yaw_disturbance = 1.3
            elif key == ord("H"):
                roll_disturbance = -1.0
            elif key == ord("L"):
                roll_disturbance = 1.0
            elif key == ord("K"):
                target_altitude += 0.05
                print(f"{target_altitude=}")
            elif key == ord("J"):
                target_altitude -= 0.05
                print(f"{target_altitude=}")
        front_left_in, front_right_in, rear_left_in, rear_right_in = (
            produce_motor_velocities(
                roll,
                pitch,
                roll_velocity,
                pitch_velocity,
                roll_disturbance,
                pitch_disturbance,
                yaw_disturbance,
                altitude,
                target_altitude,
            )
        )
        front_left_motor.setVelocity(front_left_in)
        front_right_motor.setVelocity(front_right_in)
        rear_left_motor.setVelocity(rear_left_in)
        rear_right_motor.setVelocity(rear_right_in)


if __name__ == "__main__":
    main()
