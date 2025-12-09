import math
from typing import final

import numpy as np
from controller.motor import Motor
from controller.robot import Robot
from numpy.typing import NDArray

from const import (
    INITIAL_THRUST,
    K_PITCH_P,
    K_ROLL_P,
    K_VERTICAL_OFFSET,
    K_VERTICAL_P,
    K_VERTICAL_THRUST,
    LEFT_COMPENSATION,
    REAR_COMPENSATION,
    TRAVEL_VARIANCE,
    Coordinate
)
from utils import clamp


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
        self.velodyne = self.robot.getLidar("velodyne")
        self.__enable_components()

    def __enable_components(self):
        self.gyro.enable(self.timestep)
        self.compass.enable(self.timestep)
        self.gps.enable(self.timestep)
        self.camera.enable(self.timestep)
        self.imu.enable(self.timestep)
        self.keyboard.enable(self.timestep)
        self.velodyne.enable(self.timestep)
        self.velodyne.enablePointCloud()
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

    def move_to_coord(self, dst: Coordinate) -> bool:
        """
        Implement the simplest navigation strategy which is to align with the target
        on the x-y plane and move forwards toward it. Then align the altitude (z).

        Returns:
            True if we've reached an acceptable proximity to the destination else False
        """
        x, y, z = self.gps.getValues()
        dst_x, dst_y, dst_z = dst
        dx, dy, dz = x - dst_x, y - dst_y, z - dst_z
        # XXX: God knows if using target alt and yaw at the same time is a good idea
        aoa = math.asin(dy / dx)
        print(f"move_to_coord() {dx=} {dy=} {dz=} {aoa=}")
        self.move(0, 0, aoa, dz)
        return (dx ** 2 + dy ** 2 + dz ** 2) <= TRAVEL_VARIANCE

    @property
    def image_array(self) -> NDArray[np.uint64]:
        """
        Getter for the camera image as a numpy array

        Returns:
            3D numpy array of rgb pixel values
        """
        return np.array(self.camera.getImageArray())

    def detect_hazard(self): ...

    def step(self, timestep: int = 0):
        return self.robot.step(timestep or self.timestep)
