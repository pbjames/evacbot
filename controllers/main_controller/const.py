# Empirically found constants ripped from the source controller
from typing import Final,Tuple
from pathlib import Path

# INFO: Types
Coordinate = Tuple[float, float, float]
BoundingBox = Tuple[Coordinate, Coordinate]

# INFO: Misc
WEBOTS_TERMINATE: Final[int] = -1

# INFO: Movement
K_VERTICAL_THRUST: Final[float] = 68.5  # with this thrust, the drone lifts.
K_VERTICAL_OFFSET: Final[float] = (
    0.6  # Vertical offset where the robot actually targets to stabilize itself.)
)
K_VERTICAL_P: Final[float] = 3.0  # P constant of the vertical PID.
K_ROLL_P: Final[float] = 50.0  # P constant of the roll PID.
K_PITCH_P: Final[float] = 30.0  # P constant of the pitch PID.
INITIAL_THRUST: Final[float] = 1.0
REAR_COMPENSATION: Final[float] = 0.996
LEFT_COMPENSATION: Final[float] = 0.998

# INFO: Pathing
ORIGIN: Final[Coordinate] = (0, 0, 0)
DRONE_BOUNDING_BOX: Final[BoundingBox] = (-0.25, -0.25, -0.20), (0.25, 0.25, 0.20)
TRAVEL_VARIANCE: Final[float] = 0.78**2
SAMPLE_ATTEMPTS: Final[int] = 200

# INFO: CV
MODEL_PATH: Final[Path] = Path(".") / "best.pt"