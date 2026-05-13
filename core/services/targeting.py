from typing import TYPE_CHECKING, Callable, Optional
from wpimath.geometry import Pose2d, Pose3d
from wpimath.kinematics import ChassisSpeeds
from lib import logger, utils
from lib.classes import Alliance, Zone
from core.classes import Target
import core.constants as constants

class Targeting():
  def __init__(
      self,
      getRobotPose: Callable[[], Pose2d],
      getChassisSpeeds: Callable[[], ChassisSpeeds]
    ) -> None:
    self._constants = constants.Services.Targeting
    self._getRobotPose = getRobotPose
    self._getChassisSpeeds = getChassisSpeeds

    self._alliance: Optional[Alliance] = None
    self._targets: dict[Target, Pose3d] = {}
    self._targetZones: dict[Target, Zone] = {}

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTargets()
    self._updateTelemetry()

  def _updateTargets(self) -> None:
    if utils.getAlliance() != self._alliance:
      self._alliance = utils.getAlliance()
      self._targets = constants.Game.Field.Targets.TARGETS[self._alliance]
      self._targetZones = constants.Game.Field.Targets.TARGET_ZONES[self._alliance]

  def getTargetPose(self, target: Target) -> Pose3d:
    return self._targets.get(target, Pose3d(self._getRobotPose()))
  
  def getNearestTargetPose(self, targets: list[Target]) -> Pose3d:
    return Pose3d(self._getRobotPose()).nearest([self._targets[target] for target in self._targets if target in targets])

  def _updateTelemetry(self) -> None:
    pass
