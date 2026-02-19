from typing import Optional
from wpilib import DriverStation, SmartDashboard
from lib.classes import Alliance
from lib import logger, utils
from core.classes import MatchState, HubState

class Match():
  def __init__(self) -> None:
    self._selectedAlliance: Optional[Alliance] = None

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateMatch()
    self._updateTelemetry()

  def _updateMatch(self) -> None:
    match DriverStation.getGameSpecificMessage()[:1]:
      case "R": self._selectedAlliance = Alliance.Red
      case "B": self._selectedAlliance = Alliance.Blue
      case _: self._selectedAlliance = None

    # TODO: add match timing logic and state checks for each shift
    # ======================
    # AUTO: 20s (20 -> 0) - HUB ACTIVE
    # TRANSITION: 10s (140 -> 130) - HUB ACTIVE
    # SHIFT 1: 25s (130 -> 105) - selected alliance: HUB INACTIVE
    # SHIFT 2: 25s (105 -> 80) - selected alliance: HUB ACTIVE
    # SHIFT 3: 25s (80 -> 55) - selected alliance: HUB INACTIVE
    # SHIFT 4: 25s (55 -> 30) - selected alliance: HUB ACTIVE
    # END GAME: 30s (30 -> 0) - HUB ACTIVE
    # ======================

  def _updateTelemetry(self) -> None:
    SmartDashboard.putString("Match/SelectedAlliance", self._selectedAlliance.name if self._selectedAlliance is not None else "None")
