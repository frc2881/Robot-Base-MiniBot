from enum import Enum, auto
from dataclasses import dataclass

class AutoPath(Enum):
  CUSTOM = auto()

class Target(Enum):
  DEFAULT = auto()

class Zone(Enum):
  DEFAULT = auto()

class MatchState(Enum):
  STOPPED = auto()
  AUTO = auto()
  TELEOP = auto()
  END_GAME = auto()

class LightsMode(Enum):
  DEFAULT = auto()
  ROBOT_NOT_CONNECTED = auto()
  ROBOT_NOT_HOMED = auto()
  ROBOT_IS_HOMING = auto()
  VISION_NOT_READY = auto()
