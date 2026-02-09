from typing import Optional
from wpilib import DriverStation, SmartDashboard
from lib.classes import Alliance
from lib import logger, utils

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

  def _updateTelemetry(self) -> None:
    SmartDashboard.putString("Match/SelectedAlliance", self._selectedAlliance.name if self._selectedAlliance is not None else "None")
