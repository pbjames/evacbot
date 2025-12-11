from argparse import ArgumentParser
import math
import struct
from const import ORIGIN, WEBOTS_TERMINATE
from mavic import Mavic
from pathing import Pathing
from utils import point_cloud_filter, print_help


def main():
    parser = ArgumentParser()
    parser.add_argument("-v", "--verbose", action="store_true")
    parser.add_argument("-m", "--manual", action="store_true")
    parsed_args = parser.parse_args()
    manual = parsed_args.manual  # pyright: ignore[reportAny]
    verbose = parsed_args.verbose  # pyright: ignore[reportAny]

    def verbose_log(s: str):
        if verbose:
            print(s)

    def manual_control(mavic: Mavic):
        """
        Use this mode by adding '-m' to the controllerArgs string list in the scene
        tree view of the robot. It's included for testing of movement by controlling
        the robot with the keyboard.
        """
        target_altitude = 1
        while mavic.step() != WEBOTS_TERMINATE:
            mavic.blink_switch_leds(int(mavic.robot.time) % 2 == 0)
            roll_disturbance = pitch_disturbance = yaw_disturbance = 0.0
            while (key := mavic.keyboard.getKey()) > 0:
                key_char = chr(key)
                if key_char == "K":
                    pitch_disturbance = -2.0
                elif key_char == "J":
                    pitch_disturbance = 2.0
                elif key_char == "L":
                    yaw_disturbance = -1.3
                elif key_char == "H":
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
            rotational_values = roll_disturbance, pitch_disturbance, yaw_disturbance
            mavic.move(*rotational_values, target_altitude)
            print(f"{mavic.up_sensor.getValue()=}")
            print(f"{mavic.down_sensor.getValue()=}")

    def update_pathing_position(mavic: Mavic, pathing: Pathing):
        x, y, z = mavic.gps.getValues()
        pathing.position = (x, y, z)

    def control_loop(mavic: Mavic, pathing: Pathing):
        """
        1. Take information from the environment and process it
        2. Compile visual hazard information
        3. Retrieve point cloud from sensors, up / down points
        4. Assess the location of hazardous objects and correlate with point cloud
        5. Create a random graph of nodes in space
        6. Assign weights to the nodes based on proximity to hazards and visited points
        8. Queue the movements necessary to get to the node and exhaust it
        """
        update_pathing_position(mavic, pathing)
        # mavic.detect_fire_hazard()
        point_cloud = mavic.velodyne.getPointCloud()
        pathing.add_points(point_cloud_filter(point_cloud))
        for point in pathing.point_cloud:
            mavic.emitter.send(struct.pack("ifff", 1, *point))
        move_to_me = pathing.sample_safe_point()
        mavic.move_to_coord(move_to_me)
        return move_to_me

    mavic = Mavic()
    verbose_log(f"[*] {manual=}")
    if manual:
        print_help()
        manual_control(mavic)
    else:
        print("[*] Using automatic mode...")
        pathing = Pathing()
        moving = False
        point = ORIGIN
        while mavic.step() != WEBOTS_TERMINATE:
            if not moving:
                print("[*] Choosing next node again")
                point = control_loop(mavic, pathing)
            moving = not mavic.move_to_coord(point)
            if moving:
                update_pathing_position(mavic, pathing)
            verbose_log(f"GPS: {mavic.gps.getValues()}")


if __name__ == "__main__":
    main()
