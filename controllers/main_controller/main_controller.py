from typing import Final, final
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
INITIAL_THRUST: Final[float] = 1.0


@final
class Mavic:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.camera = self.robot.getCamera("camera")
        self.front_left_led = self.robot.getLED("front left led")
        self.front_right_led = self.robot.getLED("front right led")
        self.imu = self.robot.getInertialUnit("inertial unit")
        self.gps = self.robot.getGPS("gps")
        self.compass = self.robot.getCompass("compass")
        self.gyro = self.robot.getGyro("gyro")
        self.front_left_motor = self.robot.getMotor("front left propeller")
        self.front_right_motor = self.robot.getMotor("front right propeller")
        self.rear_left_motor = self.robot.getMotor("rear left propeller")
        self.rear_right_motor = self.robot.getMotor("rear right propeller")
        self.keyboard = self.robot.getKeyboard()
        self.__enable_components()

    def __enable_components(self):
        self.gyro.enable(self.timestep)
        self.compass.enable(self.timestep)
        self.gps.enable(self.timestep)
        self.camera.enable(self.timestep)
        self.imu.enable(self.timestep)
        self.keyboard.enable(self.timestep)
        motors: list[Motor] = [
            self.rear_left_motor,
            self.front_left_motor,
            self.rear_right_motor,
            self.front_right_motor,
        ]
        for motor in motors:
            motor.setPosition(math.inf)
            motor.setVelocity(INITIAL_THRUST)

    def blink_switch_leds(self, value: bool):
        self.front_left_led.set(not value)
        self.front_right_led.set(value)

    def move_disturbance(
        self,
        roll_disturbance: float,
        pitch_disturbance: float,
        yaw_disturbance: float,
        target_altitude: float,
    ):
        roll, pitch, _ = self.imu.getRollPitchYaw()
        _, _, altitude = self.gps.getValues()
        roll_velocity, pitch_velocity, _ = self.gyro.getValues()
        clamped_roll = clamp(roll, -1.0, 1.0)
        roll_input = K_ROLL_P * clamped_roll + roll_velocity + roll_disturbance
        clamped_pitch = clamp(pitch, -1.0, 1.0)
        pitch_input = K_PITCH_P * clamped_pitch + pitch_velocity + pitch_disturbance
        yaw_input = yaw_disturbance
        altitude_diff = target_altitude - altitude + K_VERTICAL_OFFSET
        clamped_difference_altitude = clamp(altitude_diff, -1.0, 1.0)
        vertical_input = K_VERTICAL_P * (clamped_difference_altitude**3)
        front_left_velocity = vertical_input - roll_input + pitch_input - yaw_input
        front_right_velocity = vertical_input + roll_input + pitch_input + yaw_input
        rear_left_velocity = vertical_input - roll_input - pitch_input + yaw_input
        rear_right_velocity = vertical_input + roll_input - pitch_input - yaw_input
        self.front_left_motor.setVelocity(K_VERTICAL_THRUST + front_left_velocity)
        self.front_right_motor.setVelocity(-K_VERTICAL_THRUST - front_right_velocity)
        self.rear_left_motor.setVelocity(-K_VERTICAL_THRUST - rear_left_velocity)
        self.rear_right_motor.setVelocity(K_VERTICAL_THRUST + rear_right_velocity)

    def step(self, timestep: int = 0):
        return self.robot.step(timestep or self.timestep)


def clamp(value: float, min_v: float, max_v: float):
    return min(max_v, max(value, min_v))


def print_help():
    print(
        """
You can control the drone with your computer keyboard:
- 'up': move forward.
- 'down': move backward.
- 'right': turn right.
- 'left': turn left.
- 'shift + up': increase the target altitude.
- 'shift + down': decrease the target altitude.
- 'shift + right': strafe right.
- 'shift + left': strafe left.
You can control the drone with your computer keyboard:
- 'up': move forward.
- 'down': move backward.
- 'right': turn right.
- 'left': turn left.
- 'shift + up': increase the target altitude.
- 'shift + down': decrease the target altitude.
- 'shift + right': strafe right.
- 'shift + left': strafe left.
"""
    )


def main():
    mavic = Mavic()
    print("Starting the drone..")
    while mavic.step() != -1:
        if mavic.robot.getTime() > 1.0:
            break
    print_help()
    target_altitude = 1
    while mavic.step() != -1:
        mavic.blink_switch_leds(int(mavic.robot.time) % 2 == 0)
        roll_disturbance = pitch_disturbance = yaw_disturbance = 0.0
        while (key := mavic.keyboard.getKey()) > 0:
            if key == ord("K"):
                pitch_disturbance = -2.0
            elif key == ord("J"):
                pitch_disturbance = 2.0
            elif key == ord("L"):
                yaw_disturbance = -1.3
            elif key == ord("H"):
                yaw_disturbance = 1.3
            elif key == mavic.keyboard.LEFT:
                roll_disturbance = -1.0
            elif key == mavic.keyboard.RIGHT:
                roll_disturbance = 1.0
            elif key == mavic.keyboard.UP:
                target_altitude += 0.05
                print(f"{target_altitude=}")
            elif key == mavic.keyboard.DOWN:
                target_altitude -= 0.05
                print(f"{target_altitude=}")
        mavic.move_disturbance(
            roll_disturbance, pitch_disturbance, yaw_disturbance, target_altitude
        )


if __name__ == "__main__":
    main()
