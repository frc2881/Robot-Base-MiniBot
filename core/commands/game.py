from typing import TYPE_CHECKING
from commands2 import Command, cmd
from wpilib import RobotBase
from wpimath import units
from lib import logger, utils
from lib.classes import ControllerRumbleMode, ControllerRumblePattern
from core.classes import Target
import core.constants as constants
if TYPE_CHECKING: from core.robot import RobotCore

class Game:
  def __init__(self, robot: "RobotCore") -> None:
    self._robot = robot
    
  def alignRobotToTargetPose(self, target: Target, alignRotationOnly: bool = False) -> Command:
    return (
      self._robot.drive.alignToTargetPose(self._robot.localization.getRobotPose, lambda: self._robot.targeting.getTargetPose(target), alignRotationOnly)
      .andThen(self.rumbleControllers(ControllerRumbleMode.Driver))
      .withName(f'Game:AlignRobotToTargetPose:{ target.name }')
    )

  def alignRobotToNearestTargetPose(self, targets: list[Target], alignRotationOnly: bool = False) -> Command:
    return (
      self._robot.drive.alignToTargetPose(self._robot.localization.getRobotPose, lambda: self._robot.targeting.getNearestTargetPose(targets), alignRotationOnly)
      .andThen(self.rumbleControllers(ControllerRumbleMode.Driver))
      .withName("Game:AlignRobotToNearestTargetPose")
    )

  def alignRobotToTargetHeading(self, target: Target) -> Command:
    return (
      self._robot.drive.alignToTargetHeading(self._robot.localization.getRobotPose, lambda: self._robot.targeting.getTargetPose(target))
      .withName(f'Game:AlignRobotToTargetHeading:{ target.name }')
    )

  def alignRobotToNearestBump(self) -> Command:
    return (
      self.alignRobotToNearestTargetPose([Target.BumpLeftInOut, Target.BumpLeftOutIn, Target.BumpRightInOut, Target.BumpRightOutIn])
      .withName("Game:AlignRobotToNearestBump")
    )
  
  def resetGyro(self) -> Command:
    return (
      self._robot.gyro.reset()
      .andThen(self.rumbleControllers(ControllerRumbleMode.Driver))
      .ignoringDisable(True)
      .withName("Game:ResetGyro")
    )

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
