from mavic import Mavic
from utils import print_help


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
