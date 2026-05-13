from enum import Enum, IntEnum, auto
from dataclasses import dataclass
from wpimath import units

class Target(Enum):
  Hub = auto()
  ShuttleLeft = auto()
  ShuttleRight = auto()
  BumpLeftInOut = auto()
  BumpLeftOutIn = auto()
  BumpRightInOut = auto()
  BumpRightOutIn = auto()

class MatchState(Enum):
  Stopped = auto()
  Auto = auto()
  Transition = auto()
  Shift1 = auto()
  Shift2 = auto()
  Shift3 = auto()
  Shift4 = auto()
  EndGame = auto()

class HubState(Enum):
  Inactive = auto()
  Active = auto()

class LightsMode(Enum):
  Default = auto()
  RobotNotConnected = auto()
  RobotNotHomed = auto()
  RobotIsHoming = auto()
  VisionNotReady = auto()
