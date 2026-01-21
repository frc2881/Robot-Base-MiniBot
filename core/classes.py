from enum import Enum, auto

class Target(Enum):
  Hub = auto()
  TowerLeft = auto()
  TowerRight = auto()
  CornerLeft = auto()
  CornerRight = auto()
  Outpost = auto()
  Depot = auto()
