import math
from wpimath import units
from lib.classes import RobotState, RobotMode
from lib import logger, telemetry, utils
from core.classes import MatchState

class Match():
  def __init__(self) -> None:
    self._matchState = MatchState.Stopped
    self._matchStateTime: units.seconds = 0

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateMatch()
    self._updateTelemetry()

  def _updateMatch(self) -> None:
    if utils.getRobotState() == RobotState.Enabled:
      matchTime = utils.getMatchTime()
      if utils.getRobotMode() == RobotMode.Auto:
        self._matchState = MatchState.Auto
        self._matchStateTime = matchTime
      if utils.getRobotMode() == RobotMode.Teleop:
        self._matchState = MatchState.EndGame if utils.isValueWithinRange(matchTime, 0, 31) else MatchState.Teleop
        self._matchStateTime = matchTime
    else:
      self._matchState = MatchState.Stopped
      self._matchStateTime = 0

  def getMatchState(self) -> MatchState:
    return self._matchState
  
  def getMatchStateTime(self) -> units.seconds:
    return self._matchStateTime

  def _updateTelemetry(self) -> None:
    telemetry.log("Match/State", self.getMatchState().name)
    telemetry.log("Match/StateTime", math.floor(self.getMatchStateTime()))
