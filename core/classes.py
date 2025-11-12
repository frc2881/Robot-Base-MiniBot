from enum import Enum, auto
from dataclasses import dataclass
from wpimath.geometry import Pose3d

class TargetType(Enum):
  Reef = auto()
  CoralStation = auto()

class TargetAlignmentLocation(Enum):
  Center = auto()
  Left = auto()
  Right = auto()

@dataclass(frozen=True, slots=True)
class Target():
  type: TargetType
  pose: Pose3d
