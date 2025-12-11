from collections.abc import Generator
import struct
import time
from typing import Final
from controller.field import Field
from controller.receiver import Receiver
from controller.supervisor import Supervisor


type Coordinate = tuple[float, float, float]
WEBOTS_TERMINATE: Final[int] = -1
CHILD_INSERT_IDX: Final[int] = -1
GROUP_NAME: Final[str] = "SP_DISPLAY_GROUP"
GROUP_NODE: Final[str] = f"DEF {GROUP_NAME} Group {{ children [] }}"
BLUE_COLOUR: Final[tuple[int, int, int]] = (0, 0, 1)
RED_COLOUR: Final[tuple[int, int, int]] = (1, 0, 0)
UPDATE_FREQUENCY: Final[float] = 1.0


def setup_group(root: Field):
    root.importMFNodeFromString(CHILD_INSERT_IDX, GROUP_NODE)


def create_ball_at(root: Field, u: Coordinate, rgb: tuple[int, int, int]):
    x, y, z = u
    r, g, b = rgb
    sphere_string = f"""
    Transform {{
      translation {x} {y} {z}
      children [
        Shape {{
          appearance Appearance {{
            material Material {{
              diffuseColor {r} {g} {b}
              transparency 0.5
            }}
          }}
          geometry Sphere {{
            radius 0.05
          }}
        }}
      ]
    }}
    """
    root.importMFNodeFromString(CHILD_INSERT_IDX, sphere_string)


def clear_display_balls(root: Field):
    while root.getCount() > 0:
        root.removeMF(CHILD_INSERT_IDX)  # pyright: ignore[reportUnknownMemberType]


def get_latest_packet(rx: Receiver) -> Generator[bytes, None, None]:
    if rx.getQueueLength():
        yield rx.getBytes()
        rx.nextPacket()


def handle_packet(root: Field, mode: int, point: Coordinate):
    if mode == 0:
        create_ball_at(root, point, RED_COLOUR)
    elif mode == 1:
        create_ball_at(root, point, BLUE_COLOUR)


def main():
    supervisor = Supervisor()
    root = supervisor.getRoot().getField("children")
    receiver = supervisor.getReceiver("receiver")
    receiver.enable(int(supervisor.getBasicTimeStep()))
    last_display_update = 0
    setup_group(root)
    group = supervisor.getFromDef(GROUP_NAME).getField("children")
    while supervisor.step() != -1:
        if time.monotonic() - last_display_update > UPDATE_FREQUENCY:
            clear_display_balls(group)
            for msg in get_latest_packet(receiver):
                mode, *point = struct.unpack("ifff", msg)
                handle_packet(group, mode, point)
                last_display_update = time.monotonic()


if __name__ == "__main__":
    main()
