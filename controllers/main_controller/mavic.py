from typing import Final, final

from numpy.typing import NDArray
from controller.motor import Motor
from controller.robot import Robot
import numpy as np
import math

from utils import clamp

# Empirically found constants ripped from the source controller
K_VERTICAL_THRUST: Final[float] = 68.5  # with this thrust, the drone lifts.
K_VERTICAL_OFFSET: Final[float] = (
    0.6  # Vertical offset where the robot actually targets to stabilize itself.)
)
K_VERTICAL_P: Final[float] = 3.0  # P constant of the vertical PID.
K_ROLL_P: Final[float] = 50.0  # P constant of the roll PID.
K_PITCH_P: Final[float] = 30.0  # P constant of the pitch PID.
INITIAL_THRUST: Final[float] = 1.0

# These were found by James to hinder idle drift
REAR_COMPENSATION: Final[float] = 0.996
LEFT_COMPENSATION: Final[float] = 0.998


@final
class Mavic:
    """
    Store all the components as properties and operate on them holistically, let's us
    interop between components cleanly too.
    """
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

    def move(
        self,
        roll_disturbance: float,
        pitch_disturbance: float,
        yaw_disturbance: float,
        target_altitude: float,
    ):
        """
        We're using a disturbance algorithm to control the four propellers
        based on how they affect rotation on the principle axes, as well
        as forward and vertical velocities.

        Args:
            roll_disturbance: +ve banks left, vice versa, uses imu
            pitch_disturbance: backwards (+ve) or forwards (-ve) motion, uses imu
            yaw_disturbance: +ve rotates right, -ve rotates left, uses imu
            target_altitude: controls vertical velocity to converge on alt, uses gps
        """
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
        self.front_left_motor.setVelocity(
            K_VERTICAL_THRUST * LEFT_COMPENSATION + front_left_velocity
        )
        self.front_right_motor.setVelocity(-K_VERTICAL_THRUST - front_right_velocity)
        self.rear_left_motor.setVelocity(
            -K_VERTICAL_THRUST * REAR_COMPENSATION * LEFT_COMPENSATION
            - rear_left_velocity
        )
        self.rear_right_motor.setVelocity(
            K_VERTICAL_THRUST * REAR_COMPENSATION + rear_right_velocity
        )

    @property
    def image_array(self) -> NDArray[np.uint64]:
        """
        Getter for the camera image as a numpy array

        Returns:
            3D numpy array of rgb pixel values
        """
        return np.array(self.camera.getImageArray())
        
    def detect_hazard(self):
        ...

    def step(self, timestep: int = 0):
        return self.robot.step(timestep or self.timestep)
