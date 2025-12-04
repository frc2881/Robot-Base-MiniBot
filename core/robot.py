from commands2 import cmd
from wpilib import DriverStation, SmartDashboard
from lib import logger, utils
from lib.classes import TargetAlignmentMode
from lib.controllers.xbox import Xbox
from lib.sensors.gyro_navx2 import Gyro_NAVX2
from lib.sensors.pose import PoseSensor
from core.commands.auto import Auto
from core.commands.game import Game
from core.subsystems.drive import Drive
from core.services.localization import Localization
from core.classes import TargetAlignmentLocation
import core.constants as constants

class RobotCore:
  def __init__(self) -> None:
    self._initSensors()
    self._initSubsystems()
    self._initServices()
    self._initControllers()
    self._initCommands()
    self._initTriggers()
    self._initTelemetry()
    utils.addRobotPeriodic(self._periodic)

  def _initSensors(self) -> None:
    self.gyro = Gyro_NAVX2(constants.Sensors.Gyro.NAVX2.kComType)
    self.poseSensors = tuple(PoseSensor(c) for c in constants.Sensors.Pose.kPoseSensorConfigs)

  def _initSubsystems(self) -> None:
    self.drive = Drive(self.gyro.getHeading)
    
  def _initServices(self) -> None:
    self.localization = Localization(
      self.gyro.getRotation, 
      self.drive.getModulePositions, 
      self.poseSensors
    )

  def _initControllers(self) -> None:
    self.driver = Xbox(constants.Controllers.kDriverControllerPort, constants.Controllers.kInputDeadband)
    self.operator = Xbox(constants.Controllers.kOperatorControllerPort, constants.Controllers.kInputDeadband)
    DriverStation.silenceJoystickConnectionWarning(not utils.isCompetitionMode())
    
  def _initCommands(self) -> None:
    self.game = Game(self)
    self.auto = Auto(self)

  def _initTriggers(self) -> None:
    self._setupDriver()
    self._setupOperator()

  def _setupDriver(self) -> None:
    self.drive.setDefaultCommand(self.drive.drive(self.driver.getLeftY, self.driver.getLeftX, self.driver.getRightX))
    # self.driver.rightStick().whileTrue(cmd.none())
    self.driver.leftStick().whileTrue(self.drive.lock())
    # self.driver.leftTrigger().whileTrue(cmd.none())
    # self.driver.rightTrigger().whileTrue(cmd.none())
    self.driver.rightBumper().whileTrue(self.game.alignRobotToTarget(TargetAlignmentMode.Translation, TargetAlignmentLocation.Right))
    self.driver.leftBumper().whileTrue(self.game.alignRobotToTarget(TargetAlignmentMode.Translation, TargetAlignmentLocation.Left))
    # self.driver.povUp().whileTrue(cmd.none())
    # self.driver.povDown().whileTrue(cmd.none())
    # self.driver.povLeft().whileTrue(cmd.none())
    # self.driver.povRight().whileTrue(cmd.none())
    # self.driver.a().whileTrue(cmd.none())
    # self.driver.b().whileTrue(cmd.none())
    # self.driver.y().whileTrue(cmd.none())
    # self.driver.x().whileTrue(cmd.none())
    # self.driver.start().whileTrue(cmd.none())
    self.driver.back().whileTrue(cmd.waitSeconds(0.5).andThen(self.gyro.reset()))

  def _setupOperator(self) -> None:
    pass
    # self.operator.leftTrigger().whileTrue(cmd.none())
    # self.operator.rightTrigger().whileTrue(cmd.none())
    # self.operator.leftBumper().whileTrue(cmd.none())
    # self.operator.rightBumper().whileTrue(cmd.none())
    # self.operator.povUp().whileTrue(cmd.none())
    # self.operator.povRight().whileTrue(cmd.none())
    # self.operator.povDown().whileTrue(cmd.none())
    # self.operator.povLeft().whileTrue(cmd.none())
    # self.operator.a().whileTrue(cmd.none())
    # self.operator.b().whileTrue(cmd.none())
    # self.operator.y().whileTrue(cmd.none())
    # self.operator.x().whileTrue(cmd.none())
    # self.operator.start().whileTrue(cmd.none())
    # self.operator.back().whileTrue(cmd.none())

  def _initTelemetry(self) -> None:
    SmartDashboard.putNumber("Game/Field/Length", constants.Game.Field.kLength)
    SmartDashboard.putNumber("Game/Field/Width", constants.Game.Field.kWidth)
    SmartDashboard.putNumber("Robot/Drive/Chassis/RobotLength", constants.Subsystems.Drive.kRobotLength)
    SmartDashboard.putNumber("Robot/Drive/Chassis/RobotWidth", constants.Subsystems.Drive.kRobotWidth)
    SmartDashboard.putString("Robot/Cameras/Driver", constants.Cameras.kDriverStream)
    SmartDashboard.putStringArray("Robot/Sensors/Pose/Names", tuple(c.name for c in constants.Sensors.Pose.kPoseSensorConfigs))

  def _periodic(self) -> None:
    self._updateTelemetry()

  def disabledInit(self) -> None:
    self.reset()

  def autoInit(self) -> None:
    self.reset()

  def autoExit(self) -> None: 
    self.gyro.resetRobotToField(self.localization.getRobotPose())

  def teleopInit(self) -> None:
    self.reset()

  def testInit(self) -> None:
    self.reset()

  def simulationInit(self) -> None:
    self.reset()

  def reset(self) -> None:
    self.drive.reset()

  def _isHomed(self) -> bool:
    return (
      True
      if not utils.isCompetitionMode() else 
      True
    )
      
  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Status/IsHomed", self._isHomed())
