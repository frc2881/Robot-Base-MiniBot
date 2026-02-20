from typing import Optional
import math
from wpimath import units
from wpilib import DriverStation, SmartDashboard
from lib.classes import Alliance, RobotState, RobotMode
from lib import logger, utils
from core.classes import MatchState, HubState

class Match():
  def __init__(self) -> None:
    self._selectedAlliance: Optional[Alliance] = None
    self._matchState = MatchState.Stopped
    self._matchStateTime: units.seconds = 0
    self._hubState = HubState.Inactive

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateMatch()
    self._updateTelemetry()

  def _updateMatch(self) -> None:
    match DriverStation.getGameSpecificMessage()[:1]:
      case "R": self._selectedAlliance = Alliance.Red
      case "B": self._selectedAlliance = Alliance.Blue
      case _: self._selectedAlliance = None

    if utils.getRobotState() == RobotState.Enabled:
      matchTime = utils.getMatchTime()
      alliance = utils.getAlliance()
      if utils.getRobotMode() == RobotMode.Auto:
        self._matchState = MatchState.Auto
        self._matchStateTime = matchTime
        self._hubState = HubState.Active
      if utils.getRobotMode() == RobotMode.Teleop:
        if utils.isValueInRange(matchTime, 130, 140):
          self._matchState = MatchState.Transition
          self._matchStateTime = matchTime - 130
          self._hubState = HubState.Active
        if utils.isValueInRange(matchTime, 105, 130):
          self._matchState = MatchState.Shift1
          self._matchStateTime = matchTime - 105
          self._hubState = HubState.Active if alliance != self._selectedAlliance and self._selectedAlliance is not None else HubState.Inactive
        if utils.isValueInRange(matchTime, 80, 105):
          self._matchState = MatchState.Shift2
          self._matchStateTime = matchTime - 80
          self._hubState = HubState.Active if alliance == self._selectedAlliance and self._selectedAlliance is not None else HubState.Inactive
        if utils.isValueInRange(matchTime, 55, 80):
          self._matchState = MatchState.Shift3
          self._matchStateTime = matchTime - 55
          self._hubState = HubState.Active if alliance != self._selectedAlliance and self._selectedAlliance is not None else HubState.Inactive
        if utils.isValueInRange(matchTime, 30, 55):
          self._matchState = MatchState.Shift4
          self._matchStateTime = matchTime - 30
          self._hubState = HubState.Active if alliance == self._selectedAlliance and self._selectedAlliance is not None else HubState.Inactive
        if utils.isValueInRange(matchTime, 0, 30):
          self._matchState = MatchState.EndGame
          self._matchStateTime = matchTime
          self._hubState = HubState.Active
    else:
      self._matchState = MatchState.Stopped
      self._hubState = HubState.Inactive
      self._matchStateTime = 0

  def getMatchState(self) -> MatchState:
    return self._matchState
  
  def getMatchStateTime(self) -> units.seconds:
    return self._matchStateTime
  
  def getHubState(self) -> HubState:
    return self._hubState

  def _updateTelemetry(self) -> None:
    SmartDashboard.putString("Match/SelectedAlliance", self._selectedAlliance.name if self._selectedAlliance is not None else "None")
    SmartDashboard.putString("Match/State", self.getMatchState().name)
    SmartDashboard.putNumber("Match/StateTime", math.floor(self.getMatchStateTime()))
    SmartDashboard.putString("Match/Hub", self.getHubState().name)
