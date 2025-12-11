import math
import struct
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
    Coordinate,
    MODEL_PATH
)
from utils import clamp, normalise


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
        self.emitter = self.robot.getEmitter("emitter")
        self.up_sensor = self.robot.getDistanceSensor("ds_up")
        self.down_sensor = self.robot.getDistanceSensor("ds_down")
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
        self.up_sensor.enable(self.timestep)
        self.down_sensor.enable(self.timestep)
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
        _, _, yaw = self.imu.getRollPitchYaw()
        dst_x, dst_y, dst_z = dst
        dx, dy, dz = x - dst_x, y - dst_y, z - dst_z
        angle = normalise((math.atan2(dy, dx) - math.pi) - yaw)
        distance_from_us = dx**2 + dy**2  # + dz**2
        self.move(0, -0.50, angle, 1)
        self.emitter.send(struct.pack("fff", *dst))
        # print(f"move_to_coord() {dx=} {dy=} {dz=} {angle=} {distance_from_us=}")
        return abs(distance_from_us) <= TRAVEL_VARIANCE

    @property
    def image_array(self) -> NDArray[np.uint64]:
        """
        Getter for the camera image as a numpy array

        Returns:
            3D numpy array of rgb pixel values
        """
        return np.array(self.camera.getImageArray())

    def detect_fire_hazard(self) -> bool:
        from ultralytics import YOLO
        model = YOLO(MODEL_PATH)
        results = model(self.camera.getImage())
        for box in results.boxes:
            label = results.names[int(box.cls)]
            conf = float(box.conf)
            if label.lower() == "fire" and conf > 0.65:
                return True
        return False
        
    def step(self, timestep: int = 0):
        return self.robot.step(timestep or self.timestep)
