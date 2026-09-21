from enum import Enum, auto
from dataclasses import dataclass

class AutoPath(Enum):
  CUSTOM = auto()

class Target(Enum):
  Default = auto()

class Zone(Enum):
  Default = auto()

class MatchState(Enum):
  Stopped = auto()
  Auto = auto()
  Teleop = auto()
  EndGame = auto()

class LightsMode(Enum):
  Default = auto()
  RobotNotConnected = auto()
  RobotNotHomed = auto()
  RobotIsHoming = auto()
  VisionNotReady = auto()
