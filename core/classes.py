from enum import Enum, auto

class Target(Enum):
  Hub = auto()
  TowerLeft = auto()
  TowerRight = auto()
  TrenchLeft = auto()
  TrenchRight = auto()
  CornerLeft = auto()
  CornerRight = auto()
  Outpost = auto()
  Depot = auto()
