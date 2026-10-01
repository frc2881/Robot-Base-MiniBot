from wpimath import units
from lib import logger, telemetry, utils
from lib.classes import RobotState, RobotMode
from core.classes import MatchState

class Match():
  def __init__(self) -> None:
    self._matchState = MatchState.STOPPED
    self._matchStateTime: units.seconds = 0

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateMatch()
    self._updateTelemetry()

  def _updateMatch(self) -> None:
    if utils.getRobotState() == RobotState.ENABLED:
      matchTime = utils.getMatchTime()
      if utils.getRobotMode() == RobotMode.AUTO:
        self._matchState = MatchState.AUTO
        self._matchStateTime = matchTime
      if utils.getRobotMode() == RobotMode.TELEOP:
        self._matchState = MatchState.END_GAME if utils.isValueWithinRange(matchTime, 0, 31) else MatchState.TELEOP
        self._matchStateTime = matchTime
    else:
      self._matchState = MatchState.STOPPED
      self._matchStateTime = 0

  def getMatchState(self) -> MatchState:
    return self._matchState
  
  def getMatchStateTime(self) -> units.seconds:
    return self._matchStateTime

  def _updateTelemetry(self) -> None:
    telemetry.log("Match/Time",  utils.getMatchTime())
    telemetry.log("Match/State", self.getMatchState().name)
    telemetry.log("Match/StateTime", self.getMatchStateTime())
