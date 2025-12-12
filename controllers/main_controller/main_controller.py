from argparse import ArgumentParser
import math

from const import ORIGIN
from mavic import Mavic
from pathing import Pathing
from utils import point_cloud_filter, print_help


WEBOTS_TERMINATE = -1  # robot.step() returns -1 on exit


def main():
    parser = ArgumentParser()
    parser.add_argument("-v", "--verbose", action="store_true")
    parser.add_argument("-m", "--manual", action="store_true")
    parsed_args = parser.parse_args()

    manual: bool = bool(parsed_args.manual)
    verbose: bool = bool(parsed_args.verbose)

    def verbose_log(s: str):
        if verbose:
            print(s)

    def manual_control(mavic: Mavic):
        """
        Manual control mode (-m).
        Controls (Webots keyboard):
          - Arrow Up/Down: pitch forward/back
          - Arrow Left/Right: yaw
          - Shift + Left/Right: roll (strafe)
          - PageUp/PageDown: altitude up/down
          - K: enable hazard detection (optional hook)
          - L: disable hazard detection (optional hook)
        """
        target_altitude = 1.0
        detection_enabled = False

        while mavic.step() != WEBOTS_TERMINATE:
            mavic.blink_switch_leds(int(mavic.robot.time) % 2 == 0)

            roll_disturbance = 0.0
            pitch_disturbance = 0.0
            yaw_disturbance = 0.0

            key = mavic.keyboard.getKey()
            while key > 0:
                # Movement (arrows)
                if key == mavic.keyboard.UP:
                    pitch_disturbance = -2.0
                elif key == mavic.keyboard.DOWN:
                    pitch_disturbance = 2.0
                elif key == mavic.keyboard.LEFT:
                    yaw_disturbance = 1.3
                elif key == mavic.keyboard.RIGHT:
                    yaw_disturbance = -1.3

                # Roll / strafing (shift + arrows)
                elif key == (mavic.keyboard.SHIFT + mavic.keyboard.LEFT):
                    roll_disturbance = 1.0
                elif key == (mavic.keyboard.SHIFT + mavic.keyboard.RIGHT):
                    roll_disturbance = -1.0

                # Altitude (page up/down) – more reliable than SHIFT+UP/DOWN
                elif key == mavic.keyboard.PAGEUP:
                    target_altitude += 0.05
                    print(f"target_altitude={target_altitude:.2f}")
                elif key == mavic.keyboard.PAGEDOWN:
                    target_altitude -= 0.05
                    print(f"target_altitude={target_altitude:.2f}")

                # Optional toggles (printable keys come through as ASCII)
                elif key == ord("k") or key == ord("K"):
                    detection_enabled = True
                    print("Hazard detection ENABLED.")
                elif key == ord("l") or key == ord("L"):
                    detection_enabled = False
                    print("Hazard detection DISABLED.")

                key = mavic.keyboard.getKey()

            # IMPORTANT: motors update every tick
            mavic.move(roll_disturbance, pitch_disturbance, yaw_disturbance, target_altitude)

            # Optional hazard hook (only if you have it implemented)
            if detection_enabled:
                mavic.detect_fire_hazard()

            _, _, yaw = mavic.imu.getRollPitchYaw()
            verbose_log(f"[+] yaw/pi={(yaw / math.pi):.3f}")

    def update_pathing_position(mavic: Mavic, pathing: Pathing):
        x, y, z = mavic.gps.getValues()
        pathing.position = (x, y, z)

    def control_loop(mavic: Mavic, pathing: Pathing):
        """
        Compute the next destination point to travel to.
        Do NOT call move_to_coord() here (that should happen in the main loop).
        """
        update_pathing_position(mavic, pathing)

        # If you have this method, keep it. Otherwise comment it out.
        # mavic.detect_hazard()

        point_cloud = mavic.velodyne.getPointCloud()
        pathing.add_points(point_cloud_filter(point_cloud))

        move_to_me = pathing.sample_random_point()
        return move_to_me

    mavic = Mavic()
    verbose_log(f"[*] manual={manual}")

    if manual:
        print_help()
        manual_control(mavic)
        return

    # Automatic mode
    print("[*] Using automatic mode...")
    pathing = Pathing()

    moving = False
    point = ORIGIN

    while mavic.step() != WEBOTS_TERMINATE:
        if not moving:
            print("[*] Choosing next node again")
            point = control_loop(mavic, pathing)

        # Move toward the selected point
        reached = mavic.move_to_coord(point)
        moving = not reached

        if moving:
            update_pathing_position(mavic, pathing)

        verbose_log(f"GPS: {mavic.gps.getValues()}")


if __name__ == "__main__":
    main()