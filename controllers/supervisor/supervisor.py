import struct
import time
from typing import Final
from controller.field import Field
from controller.receiver import Receiver
from controller.supervisor import Supervisor


type Coordinate = tuple[float, float, float]
WEBOTS_TERMINATE: Final[int] = -1
CHILD_INSERT_IDX: Final[int] = -1


def create_ball_at(root: Field, u: Coordinate):
    x, y, z = u
    sphere_string = f"""
    DEF BALL Transform {{
      translation {x} {y} {z}
      children [
        Shape {{
          appearance Appearance {{
            material Material {{
              diffuseColor 1 0 0
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


def delete_current_ball(supervisor: Supervisor, root: Field):
    parent = supervisor.getFromDef("BALL")
    if root.getCount() > 0 and parent:
        root.removeMF(CHILD_INSERT_IDX)  # pyright: ignore[reportUnknownMemberType]


def get_latest_packet(rx: Receiver):
    if rx.getQueueLength() < 1:
        return b""
    while rx.getQueueLength() > 1:
        rx.getBytes()
        rx.nextPacket()
    return rx.getBytes()


def main():
    supervisor = Supervisor()
    root = supervisor.getRoot().getField("children")
    receiver = supervisor.getReceiver("receiver")
    receiver.enable(int(supervisor.getBasicTimeStep()))
    ball_active_t = 0
    while supervisor.step() != -1:
        if time.monotonic() - ball_active_t > 1:
            delete_current_ball(supervisor, root)
            if msg := get_latest_packet(receiver):
                point = struct.unpack("fff", msg)
                ball_active_t = time.monotonic()
                create_ball_at(root, point)


if __name__ == "__main__":
    main()
