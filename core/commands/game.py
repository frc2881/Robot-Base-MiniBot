from typing import TYPE_CHECKING
from commands2 import Command, cmd
from wpilib import RobotBase
from wpimath.geometry import Pose2d, Pose3d
from lib import logger, utils
from lib.classes import TargetAlignmentMode, ControllerRumbleMode, ControllerRumblePattern
if TYPE_CHECKING: from core.robot import RobotCore
from core.classes import TargetAlignmentLocation

class Game:
  def __init__(
      self,
      robot: "RobotCore"
    ) -> None:
    self._robot = robot

    self._testTargetPose = Pose3d(Pose2d(4.623, 4.032, 0))

  def lockRobotToTarget(self) -> Command:
    return (
      self._robot.drive.lockToTarget(self._robot.localization.getRobotPose, self._testTargetPose)
      .withName(f'Game:LockRobotToTarget')
    )

  def alignRobotToTarget(self, targetAlignmentMode: TargetAlignmentMode, targetAlignmentLocation: TargetAlignmentLocation) -> Command:
    return (
      self._robot.drive.alignToTarget(
        self._robot.localization.getRobotPose, 
        lambda: self._robot.localization.getTargetPose(targetAlignmentLocation),
        targetAlignmentMode)
      .andThen(self.rumbleControllers(ControllerRumbleMode.Driver))
      .withName(f'Game:AlignRobotToTarget:{ targetAlignmentMode.name }:{ targetAlignmentLocation.name }')
    )
  
  def isRobotAlignedToTarget(self) -> bool:
    return self._robot.drive.isAlignedToTarget()

  def rumbleControllers(
    self, 
    mode: ControllerRumbleMode = ControllerRumbleMode.Both, 
    pattern: ControllerRumblePattern = ControllerRumblePattern.Short
  ) -> Command:
    return cmd.parallel(
      self._robot.driver.rumble(pattern).onlyIf(lambda: mode != ControllerRumbleMode.Operator),
      self._robot.operator.rumble(pattern).onlyIf(lambda: mode != ControllerRumbleMode.Driver)
    ).onlyIf(
      lambda: RobotBase.isReal() and not utils.isAutonomousMode()
    ).withName(f'Game:RumbleControllers:{ mode.name }:{ pattern.name }')
