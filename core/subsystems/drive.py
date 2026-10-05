from typing import Callable, Optional
from wpimath import units
from wpimath.controller import PIDController, ProfiledPIDControllerRadians, HolonomicDriveController
from wpimath.trajectory import TrapezoidProfileRadians
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Rotation2d, Pose2d, Pose3d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition, SwerveModuleState, SwerveDrive4Kinematics
from commands2 import Subsystem, Command, cmd
from pathplannerlib.util import DriveFeedforwards
from lib import logger, telemetry, utils
from lib.classes import State, Position, IdleMode, SpeedMode, DriveOrientation, SwerveDriveModuleLocation
from lib.modules.swerve_drive import SwerveDriveModule
import core.constants as constants

class Drive(Subsystem):
  def __init__(
      self, 
      getGyroHeading: Callable[[], units.degrees]
    ) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Drive
    self._getGyroHeading = getGyroHeading

    self._telemetryName = "Robot/Subsystems/Drive"
    
    self._modules = (
      SwerveDriveModule(self._constants.SWERVE_DRIVE_MODULE_CONFIGS[SwerveDriveModuleLocation.FRONT_LEFT]),
      SwerveDriveModule(self._constants.SWERVE_DRIVE_MODULE_CONFIGS[SwerveDriveModuleLocation.FRONT_RIGHT]),
      SwerveDriveModule(self._constants.SWERVE_DRIVE_MODULE_CONFIGS[SwerveDriveModuleLocation.REAR_LEFT]),
      SwerveDriveModule(self._constants.SWERVE_DRIVE_MODULE_CONFIGS[SwerveDriveModuleLocation.REAR_RIGHT])
    )
    self._modulesLockPosition = Position.UNLOCKED

    self._driftCorrectionState = State.STOPPED
    self._driftCorrectionController = PIDController(*self._constants.DRIFT_CORRECTION_CONSTANTS.rotationControlPID)
    self._driftCorrectionController.setTolerance(self._constants.DRIFT_CORRECTION_CONSTANTS.rotationPositionTolerance)
    self._driftCorrectionController.enableContinuousInput(-180.0, 180.0)

    self._targetHeadingAlignmentState = State.STOPPED
    self._targetHeadingAlignmentController = PIDController(*self._constants.TARGET_HEADING_ALIGNMENT_CONSTANTS.rotationControlPID)
    self._targetHeadingAlignmentController.setTolerance(self._constants.TARGET_HEADING_ALIGNMENT_CONSTANTS.rotationPositionTolerance)
    self._targetHeadingAlignmentController.enableContinuousInput(-180.0, 180.0)
    self._targetHeadingAlignmentRotationInput: units.percent = 0

    self._targetPose: Optional[Pose2d] = None
    self._targetPoseAlignmentState = State.STOPPED
    self._targetPoseAlignmentController = HolonomicDriveController(
      PIDController(*self._constants.TARGET_POSE_ALIGNMENT_CONSTANTS.translationControlPID),
      PIDController(*self._constants.TARGET_POSE_ALIGNMENT_CONSTANTS.translationControlPID),
      ProfiledPIDControllerRadians(
        *self._constants.TARGET_POSE_ALIGNMENT_CONSTANTS.rotationControlPID, 
        TrapezoidProfileRadians.Constraints(units.degreesToRadians(self._constants.TARGET_POSE_ALIGNMENT_CONSTANTS.rotationMaxVelocity), units.degreesToRadians(self._constants.TARGET_POSE_ALIGNMENT_CONSTANTS.rotationMaxVelocity / 2))
      )
    )
    self._targetPoseAlignmentController.setTolerance(Pose2d(
      self._constants.TARGET_POSE_ALIGNMENT_CONSTANTS.translationPositionTolerance, 
      self._constants.TARGET_POSE_ALIGNMENT_CONSTANTS.translationPositionTolerance, 
      Rotation2d.fromDegrees(self._constants.TARGET_POSE_ALIGNMENT_CONSTANTS.rotationPositionTolerance))
    )
    self._targetPoseAlignmentController.getThetaController().enableContinuousInput(units.degreesToRadians(-180.0), units.degreesToRadians(180.0))

    self._translationXInputLimiter = SlewRateLimiter(self._constants.INPUT_RATE_LIMIT_DEMO)
    self._translationYInputLimiter = SlewRateLimiter(self._constants.INPUT_RATE_LIMIT_DEMO)
    self._rotationInputLimiter = SlewRateLimiter(self._constants.INPUT_RATE_LIMIT_DEMO)

    telemetry.log(f'{self._telemetryName}/Bumper/Length', self._constants.BUMPER_LENGTH)
    telemetry.log(f'{self._telemetryName}/Bumper/Width', self._constants.BUMPER_WIDTH)

  def periodic(self) -> None:
    self._updateTelemetry()

  def drive(self, getTranslationXInput: Callable[[], units.percent], getTranslationYInput: Callable[[], units.percent], getRotationInput: Callable[[], units.percent]) -> Command:
    return self.run(
      lambda: self._runDrive(getTranslationXInput(), getTranslationYInput(), getRotationInput())
    ).onlyIf(
      lambda: self._modulesLockPosition == Position.UNLOCKED
    ).withName("Drive:Drive")

  def _runDrive(self, translationXInput: units.percent, translationYInput: units.percent, rotationInput: units.percent) -> None:
    if self._targetHeadingAlignmentState == State.RUNNING:
      rotationInput = self._targetHeadingAlignmentRotationInput
    else:
      if self._constants.DRIFT_CORRECTION == State.ENABLED:
        isTranslating: bool = translationXInput != 0 or translationYInput != 0
        isRotating: bool = rotationInput != 0
        if isTranslating and not isRotating and not self._driftCorrectionState == State.RUNNING:
          self._driftCorrectionState = State.RUNNING
          self._driftCorrectionController.reset()
          self._driftCorrectionController.setSetpoint(self._getGyroHeading())
        elif isRotating or not isTranslating:
          self._driftCorrectionState = State.STOPPED
        if self._driftCorrectionState == State.RUNNING:
          rotationInput = self._driftCorrectionController.calculate(self._getGyroHeading())
          if self._driftCorrectionController.atSetpoint():
            rotationInput = 0
  
    if self._constants.SPEED_MODE == SpeedMode.DEMO:
      translationXInput = self._translationXInputLimiter.calculate(translationXInput * self._constants.INPUT_LIMIT_DEMO) if translationXInput != 0 else 0
      translationYInput = self._translationYInputLimiter.calculate(translationYInput * self._constants.INPUT_LIMIT_DEMO) if translationYInput != 0 else 0
      rotationInput = self._rotationInputLimiter.calculate(rotationInput * self._constants.INPUT_LIMIT_DEMO) if rotationInput != 0 else 0

    translationXVelocity: units.meters_per_second = translationXInput * self._constants.TRANSLATION_MAX_VELOCITY
    translationYVelocity: units.meters_per_second = translationYInput * self._constants.TRANSLATION_MAX_VELOCITY
    rotationVelocity: units.degrees_per_second = rotationInput * self._constants.ROTATION_MAX_VELOCITY
    
    self.setChassisSpeeds(
      ChassisSpeeds.fromFieldRelativeSpeeds(translationXVelocity, translationYVelocity, units.degreesToRadians(rotationVelocity), Rotation2d.fromDegrees(self._getGyroHeading()))
      if self._constants.DRIVE_ORIENTATION == DriveOrientation.FIELD else
      ChassisSpeeds(translationXVelocity, translationYVelocity, units.degreesToRadians(rotationVelocity))
    )

  def setChassisSpeeds(self, chassisSpeeds: ChassisSpeeds, driveFeedforwards: Optional[DriveFeedforwards] = None) -> None:
    self._setModuleStates(chassisSpeeds)

  def getChassisSpeeds(self) -> ChassisSpeeds:
    return self._constants.SWERVE_DRIVE_KINEMATICS.toChassisSpeeds(self._getModuleStates())

  def getModulePositions(self) -> tuple[SwerveModulePosition, SwerveModulePosition, SwerveModulePosition, SwerveModulePosition]:
    return (
      self._modules[SwerveDriveModuleLocation.FRONT_LEFT].getPosition(), 
      self._modules[SwerveDriveModuleLocation.FRONT_RIGHT].getPosition(), 
      self._modules[SwerveDriveModuleLocation.REAR_LEFT].getPosition(), 
      self._modules[SwerveDriveModuleLocation.REAR_RIGHT].getPosition()
    )

  def _setModuleStates(self, chassisSpeeds: ChassisSpeeds) -> None: 
    swerveModuleStates = SwerveDrive4Kinematics.desaturateWheelSpeeds(
      self._constants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
        ChassisSpeeds.discretize(
          self._constants.SWERVE_DRIVE_KINEMATICS.toChassisSpeeds(
            SwerveDrive4Kinematics.desaturateWheelSpeeds(
              self._constants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds), 
              self._constants.TRANSLATION_MAX_VELOCITY
            )), 0.02
        )
      ), self._constants.TRANSLATION_MAX_VELOCITY
    )
    for index, module in enumerate(self._modules):
      module.setTargetState(swerveModuleStates[index])

    if self._targetPoseAlignmentState != State.RUNNING:
      if chassisSpeeds.vx != 0 or chassisSpeeds.vy != 0 or chassisSpeeds.omega != 0:
        self._targetPoseAlignmentState = State.STOPPED

  def _getModuleStates(self) -> tuple[SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState]:
    return (
      self._modules[SwerveDriveModuleLocation.FRONT_LEFT].getState(), 
      self._modules[SwerveDriveModuleLocation.FRONT_RIGHT].getState(), 
      self._modules[SwerveDriveModuleLocation.REAR_LEFT].getState(), 
      self._modules[SwerveDriveModuleLocation.REAR_RIGHT].getState()
    )

  def _setIdleMode(self, idleMode: IdleMode) -> None:
    for module in self._modules: module.setIdleMode(idleMode)
    telemetry.log(f'{self._telemetryName}/IdleMode', idleMode.name)

  def holdCoastMode(self) -> Command:
    return self.startEnd(
      lambda: self._setIdleMode(IdleMode.COAST),
      lambda: self._setIdleMode(IdleMode.BRAKE)
    ).withName("Drive:HoldCoastMode")

  def lockSwerveModules(self) -> Command:
    return self.startEnd(
      lambda: self._setSwerveModulesLockPosition(Position.LOCKED),
      lambda: self._setSwerveModulesLockPosition(Position.UNLOCKED)
    ).withName("Drive:LockSwerveModules")
  
  def _setSwerveModulesLockPosition(self, position: Position) -> None:
    self._modulesLockPosition = position
    if position == Position.LOCKED:
      for index, module in enumerate(self._modules): 
        module.setTargetState(SwerveModuleState(0, Rotation2d.fromDegrees(45 if index in { 0, 3 } else -45)))

  def alignToTargetPose(self, getRobotPose: Callable[[], Pose2d], getTargetPose: Callable[[], Pose3d], alignRotationOnly: bool = False) -> Command:
    return self.startRun(
      lambda: self._initTargetPoseAlignment(getTargetPose(), getRobotPose(), alignRotationOnly),
      lambda: self._runTargetPoseAlignment(getRobotPose())
    ).until(
      lambda: self._targetPoseAlignmentState == State.COMPLETED
    ).finallyDo(
      lambda end: self._endTargetPoseAlignment()
    )
  
  def _initTargetPoseAlignment(self, targetPose: Pose3d, robotPose: Pose2d, alignRotationOnly: bool) -> None:
    self._targetPose = Pose2d(robotPose.translation(), targetPose.toPose2d().rotation()) if alignRotationOnly else targetPose.toPose2d()
    self._targetPoseAlignmentState = State.RUNNING

  def _runTargetPoseAlignment(self, robotPose: Pose2d) -> None:
    if self._targetPose is not None:
      self._setModuleStates(
        utils.clampTranslationVelocity(
          self._targetPoseAlignmentController.calculate(robotPose, self._targetPose, 0, self._targetPose.rotation()), 
          self._constants.TARGET_POSE_ALIGNMENT_CONSTANTS.translationMaxVelocity
        )
      )
      if self._targetPoseAlignmentController.atReference():
        self._targetPoseAlignmentState = State.COMPLETED

  def _endTargetPoseAlignment(self) -> None:
    self._setModuleStates(ChassisSpeeds())
    if self._targetPoseAlignmentState != State.COMPLETED:
      self._targetPoseAlignmentState = State.STOPPED

  def isAlignedToTargetPose(self) -> bool:
    return self._targetPoseAlignmentState == State.COMPLETED

  def alignToTargetHeading(self, getRobotPose: Callable[[], Pose2d], getTargetPose: Callable[[], Pose3d]) -> Command:
    return cmd.startRun(
      lambda: self._initTargetHeadingAlignment(getTargetPose()),
      lambda: self._runTargetHeadingAlignment(getRobotPose())
    ).finallyDo(
      lambda end: self._endTargetHeadingAlignment()
    )

  def _initTargetHeadingAlignment(self, targetPose: Pose3d) -> None:
    self._targetPose = targetPose.toPose2d()
    self._targetHeadingAlignmentController.reset()
    self._targetHeadingAlignmentState = State.RUNNING

  def _runTargetHeadingAlignment(self, robotPose: Pose2d) -> None:
    if self._targetPose is not None:
      self._targetHeadingAlignmentController.setSetpoint(utils.wrapAngle(utils.getTargetHeading(robotPose, self._targetPose)))
      self._targetHeadingAlignmentRotationInput = self._targetHeadingAlignmentController.calculate(robotPose.rotation().degrees()) if not self._targetHeadingAlignmentController.atSetpoint() else 0

  def _endTargetHeadingAlignment(self) -> None:
    self._targetHeadingAlignmentState = State.STOPPED
    self._targetHeadingAlignmentRotationInput = 0

  def isAlignedToTargetHeading(self) -> bool:
    return self._targetHeadingAlignmentState == State.RUNNING and self._targetHeadingAlignmentController.atSetpoint()
  
  def reset(self) -> None:
    self.setChassisSpeeds(ChassisSpeeds())
    self._driftCorrectionState = State.STOPPED
    self._targetPoseAlignmentState = State.STOPPED
    self._targetHeadingAlignmentState = State.STOPPED
    self._targetPose = None

  def _updateTelemetry(self) -> None:
    telemetry.log(f'{self._telemetryName}/Modules/States', list(self._getModuleStates()), element_type = SwerveModuleState)
    telemetry.log(f'{self._telemetryName}/TargetPoseAlignmentState', self._targetPoseAlignmentState.name)
    telemetry.log(f'{self._telemetryName}/IsAlignedToTargetPose', self.isAlignedToTargetPose())
    telemetry.log(f'{self._telemetryName}/TargetHeadingAlignmentState', self._targetHeadingAlignmentState.name)
    telemetry.log(f'{self._telemetryName}/IsAlignedToTargetHeading', self.isAlignedToTargetHeading())
    telemetry.log(f'{self._telemetryName}/Modules/LockPosition', self._modulesLockPosition.name)
