from mavic import Mavic
from utils import print_help
from argparse import ArgumentParser


def main():
    parser = ArgumentParser()
    parser.add_argument("-v", "--verbose", action="store_true")
    parser.add_argument("-m", "--manual", action="store_true")
    parsed_args = parser.parse_args()

    def verbose_log(s: str):
        if parsed_args.verbose:  # pyright: ignore[reportAny]
            print(s)

    mavic = Mavic()
    print_help()
    target_altitude = 1
    verbose_log(f"[*] {parsed_args.manual=}")  # pyright: ignore[reportAny]
    while mavic.step() != -1:
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
        verbose_log(f"{rotational_values=} {target_altitude=}")


if __name__ == "__main__":
    main()
