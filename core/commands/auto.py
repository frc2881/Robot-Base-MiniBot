from typing import TYPE_CHECKING
from enum import Enum, auto
from commands2 import Command, cmd
from wpilib import SendableChooser, SmartDashboard
from wpimath.geometry import Transform2d, Rotation2d
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath, PathConstraints, GoalEndState
from lib import logger, utils
from lib.classes import Alliance
import core.constants as constants
if TYPE_CHECKING: from core.robot import RobotCore

class AutoPath(Enum):
  CUSTOM = auto()

class Auto:
  def __init__(self, robot: "RobotCore") -> None:
    self._robot = robot

    self._paths = { path: PathPlannerPath.fromPathFile(path.name) for path in AutoPath }
    self._auto = cmd.none()

    AutoBuilder.configure(
      self._robot.localization.getRobotPose, 
      self._robot.localization.resetRobotPose,
      self._robot.drive.getChassisSpeeds, 
      self._robot.drive.setChassisSpeeds, 
      constants.Subsystems.Drive.PATHPLANNER_CONTROLLER,
      constants.Subsystems.Drive.PATHPLANNER_ROBOT_CONFIG,
      lambda: utils.getAlliance() == Alliance.Red,
      self._robot.drive
    )

    self._autos = SendableChooser()
    self._autos.setDefaultOption("0: None", self.auto_NONE)
    
    self._autos.addOption("1: Custom", self.auto_CUSTOM)

    self._autos.onChange(lambda auto: self.set(auto()))
    SmartDashboard.putData("Robot/Auto", self._autos)

  def get(self) -> Command:
    return self._auto
  
  def set(self, auto: Command) -> None:
    self._auto = auto
    SmartDashboard.putString("Robot/Auto/command", auto.getName().replace("Auto:", ""))

  def _getPath(self, path: AutoPath) -> PathPlannerPath:
    return self._paths.get(path, PathPlannerPath([], PathConstraints(0, 0, 0, 0), None, GoalEndState(0, Rotation2d())))
  
  def _reset(self, path: AutoPath) -> Command:
    return (
      AutoBuilder.resetOdom(self._getPath(path).getPathPoses()[0].transformBy(Transform2d(0, 0, self._getPath(path).getInitialHeading())))
      .andThen(cmd.waitSeconds(0.1))
    ).deadlineFor(logger.log_("Auto:Reset"))
  
  def _move(self, path: AutoPath) -> Command:
    return (
      AutoBuilder.followPath(self._getPath(path))
    ).deadlineFor(logger.log_(f'Auto:Move:{path.name}'))
  
  def auto_NONE(self) -> Command:
    return cmd.none().withName("Auto:NONE")

  def auto_CUSTOM(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.CUSTOM).deadlineFor()
    ).withName("Auto:CUSTOM")
