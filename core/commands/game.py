from typing import TYPE_CHECKING
from commands2 import Command, cmd
from wpilib import RobotBase
from lib import logger, utils
from lib.classes import TargetAlignmentMode, ControllerRumbleMode, ControllerRumblePattern
from core.classes import Target
if TYPE_CHECKING: from core.robot import RobotCore

class Game:
  def __init__(
      self,
      robot: "RobotCore"
    ) -> None:
    self._robot = robot

  def lockRobotToTarget(self, target: Target) -> Command:
    return (
      self._robot.drive.lockToTarget(
        self._robot.localization.getRobotPose, 
        lambda: self._robot.localization.getTargetPose(target))
      .withName(f'Game:LockRobotToTarget:{ target.name }')
    )

  def alignRobotToTarget(self, target: Target, targetAlignmentMode: TargetAlignmentMode) -> Command:
    return (
      self._robot.drive.alignToTarget(
        self._robot.localization.getRobotPose, 
        lambda: self._robot.localization.getTargetPose(target),
        targetAlignmentMode)
      .andThen(self.rumbleControllers(ControllerRumbleMode.Driver))
      .withName(f'Game:AlignRobotToTarget:{ target.name }:{ targetAlignmentMode.name }')
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
