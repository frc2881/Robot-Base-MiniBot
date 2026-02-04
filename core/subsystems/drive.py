from typing import Callable
from commands2 import Subsystem, Command, cmd
from wpilib import SmartDashboard, SendableChooser
from wpimath import units
from wpimath.controller import PIDController, ProfiledPIDController, ProfiledPIDControllerRadians, HolonomicDriveController
from wpimath.trajectory import TrapezoidProfile, TrapezoidProfileRadians
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Rotation2d, Pose2d, Pose3d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition, SwerveModuleState, SwerveDrive4Kinematics
from ntcore import NetworkTableInstance
from pathplannerlib.util import DriveFeedforwards
from lib import logger, utils
from lib.classes import State, Position, MotorIdleMode, SpeedMode, DriveOrientation
from lib.components.swerve_module import SwerveModule
import core.constants as constants

class Drive(Subsystem):
  def __init__(
      self, 
      getGyroHeading: Callable[[], units.degrees]
    ) -> None:
    super().__init__()
    self._getGyroHeading = getGyroHeading
    
    self._constants = constants.Subsystems.Drive

    self._modules = tuple(SwerveModule(c) for c in self._constants.SWERVE_MODULE_CONFIGS)
    self._modulesStatesPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/SmartDashboard/Robot/Drive/Modules/States", SwerveModuleState).publish()
    self._modulesLockPosition = Position.Unlocked

    self._driftCorrectionState = State.Stopped
    self._driftCorrectionController = PIDController(*self._constants.DRIFT_CORRECTION_CONSTANTS.rotationPID)
    self._driftCorrectionController.setTolerance(self._constants.DRIFT_CORRECTION_CONSTANTS.rotationPositionTolerance)
    self._driftCorrectionController.enableContinuousInput(-180.0, 180.0)

    self._targetHeadingAlignmentState = State.Stopped
    self._targetHeadingAlignmentController = PIDController(*self._constants.TARGET_HEADING_ALIGNMENT_CONSTANTS.rotationPID)
    self._targetHeadingAlignmentController.setTolerance(self._constants.TARGET_HEADING_ALIGNMENT_CONSTANTS.rotationPositionTolerance)
    self._targetHeadingAlignmentController.enableContinuousInput(-180.0, 180.0)
    self._targetHeadingAlignmentRotationInput: units.percent = 0

    self._targetPoseAlignmentState = State.Stopped
    self._targetPoseAlignmentController = HolonomicDriveController(
      PIDController(*self._constants.TARGET_POSE_ALIGNMENT_CONSTANTS.translationPID),
      PIDController(*self._constants.TARGET_POSE_ALIGNMENT_CONSTANTS.translationPID),
      ProfiledPIDControllerRadians(
        *self._constants.TARGET_POSE_ALIGNMENT_CONSTANTS.rotationPID, 
        TrapezoidProfileRadians.Constraints(self._constants.TARGET_POSE_ALIGNMENT_CONSTANTS.rotationMaxVelocity, self._constants.TARGET_POSE_ALIGNMENT_CONSTANTS.rotationMaxAcceleration)
      )
    )
    self._targetPoseAlignmentController.setTolerance(Pose2d(
      self._constants.TARGET_POSE_ALIGNMENT_CONSTANTS.translationPositionTolerance, 
      self._constants.TARGET_POSE_ALIGNMENT_CONSTANTS.translationPositionTolerance, 
      Rotation2d.fromDegrees(self._constants.TARGET_POSE_ALIGNMENT_CONSTANTS.rotationPositionTolerance))
    )
    self._targetPoseAlignmentController.getThetaController().enableContinuousInput(units.degreesToRadians(-180.0), units.degreesToRadians(180.0))

    self._targetPose: Pose3d | None = None

    self._translationXInputLimiter = SlewRateLimiter(self._constants.INPUT_RATE_LIMIT_DEMO)
    self._translationYInputLimiter = SlewRateLimiter(self._constants.INPUT_RATE_LIMIT_DEMO)
    self._rotationInputLimiter = SlewRateLimiter(self._constants.INPUT_RATE_LIMIT_DEMO)

    self._speedMode: SpeedMode = SpeedMode.Competition
    speedMode = SendableChooser()
    speedMode.setDefaultOption(SpeedMode.Competition.name, SpeedMode.Competition)
    speedMode.addOption(SpeedMode.Demo.name, SpeedMode.Demo)
    speedMode.onChange(lambda speedMode: setattr(self, "_speedMode", speedMode))
    SmartDashboard.putData("Robot/Drive/SpeedMode", speedMode)

    self._orientation: DriveOrientation = DriveOrientation.Field
    orientation = SendableChooser()
    orientation.setDefaultOption(DriveOrientation.Field.name, DriveOrientation.Field)
    orientation.addOption(DriveOrientation.Robot.name, DriveOrientation.Robot)
    orientation.onChange(lambda orientation: setattr(self, "_orientation", orientation))
    SmartDashboard.putData("Robot/Drive/Orientation", orientation)

    self._driftCorrection: State = State.Enabled
    driftCorrection = SendableChooser()
    driftCorrection.setDefaultOption(State.Enabled.name, State.Enabled)
    driftCorrection.addOption(State.Disabled.name, State.Disabled)
    driftCorrection.onChange(lambda driftCorrection: setattr(self, "_driftCorrection", driftCorrection))
    SmartDashboard.putData("Robot/Drive/DriftCorrection", driftCorrection)

    idleMode = SendableChooser()
    idleMode.setDefaultOption(MotorIdleMode.Brake.name, MotorIdleMode.Brake)
    idleMode.addOption(MotorIdleMode.Coast.name, MotorIdleMode.Coast)
    idleMode.onChange(lambda idleMode: self._setIdleMode(idleMode))
    SmartDashboard.putData("Robot/Drive/IdleMode", idleMode)

  def periodic(self) -> None:
    self._updateTelemetry()

  def drive(self, getTranslationXInput: Callable[[], units.percent], getTranslationYInput: Callable[[], units.percent], getRotationInput: Callable[[], units.percent]) -> Command:
    return self.run(
      lambda: self._runDrive(getTranslationXInput(), getTranslationYInput(), getRotationInput())
    ).onlyIf(
      lambda: self._modulesLockPosition == Position.Unlocked
    ).withName("Drive:Drive")

  def _runDrive(self, translationXInput: units.percent, translationYInput: units.percent, rotationInput: units.percent) -> None:
    if self._targetHeadingAlignmentState == State.Running:
      rotationInput = self._targetHeadingAlignmentRotationInput
    else:
      if self._driftCorrection == State.Enabled:
        isTranslating: bool = translationXInput != 0 or translationYInput != 0
        isRotating: bool = rotationInput != 0
        if isTranslating and not isRotating and not self._driftCorrectionState == State.Running:
          self._driftCorrectionState = State.Running
          self._driftCorrectionController.reset()
          self._driftCorrectionController.setSetpoint(self._getGyroHeading())
        elif isRotating or not isTranslating:
          self._driftCorrectionState = State.Stopped
        if self._driftCorrectionState == State.Running:
          rotationInput = self._driftCorrectionController.calculate(self._getGyroHeading())
          if self._driftCorrectionController.atSetpoint():
            rotationInput = 0

    if self._speedMode == SpeedMode.Demo:
      translationXInput = self._translationXInputLimiter.calculate(translationXInput * self._constants.INPUT_LIMIT_DEMO) if translationXInput != 0 else 0
      translationYInput = self._translationYInputLimiter.calculate(translationYInput * self._constants.INPUT_LIMIT_DEMO) if translationYInput != 0 else 0
      rotationInput = self._rotationInputLimiter.calculate(rotationInput * self._constants.INPUT_LIMIT_DEMO) if rotationInput != 0 else 0

    translationXVelocity: units.meters_per_second = translationXInput * self._constants.TRANSLATION_MAX_VELOCITY
    translationYVelocity: units.meters_per_second = translationYInput * self._constants.TRANSLATION_MAX_VELOCITY
    rotationVelocity: units.degrees_per_second = rotationInput * self._constants.ROTATION_MAX_VELOCITY
    
    self.setChassisSpeeds(
      ChassisSpeeds.fromFieldRelativeSpeeds(translationXVelocity, translationYVelocity, units.degreesToRadians(rotationVelocity), Rotation2d.fromDegrees(self._getGyroHeading()))
      if self._orientation == DriveOrientation.Field else
      ChassisSpeeds(translationXVelocity, translationYVelocity, units.degreesToRadians(rotationVelocity))
    )

  def setChassisSpeeds(self, chassisSpeeds: ChassisSpeeds, driveFeedforwards: DriveFeedforwards = None) -> None:
    self._setModuleStates(chassisSpeeds)

  def getChassisSpeeds(self) -> ChassisSpeeds:
    return self._constants.DRIVE_KINEMATICS.toChassisSpeeds(self._getModuleStates())

  def getModulePositions(self) -> tuple[SwerveModulePosition, ...]:
    return tuple(module.getPosition() for module in self._modules)

  def _setModuleStates(self, chassisSpeeds: ChassisSpeeds) -> None: 
    swerveModuleStates = SwerveDrive4Kinematics.desaturateWheelSpeeds(
      self._constants.DRIVE_KINEMATICS.toSwerveModuleStates(
        ChassisSpeeds.discretize(
          self._constants.DRIVE_KINEMATICS.toChassisSpeeds(
            SwerveDrive4Kinematics.desaturateWheelSpeeds(
              self._constants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds), 
              self._constants.TRANSLATION_MAX_VELOCITY
            )), 0.02
        )
      ), self._constants.TRANSLATION_MAX_VELOCITY
    )
    for index, module in enumerate(self._modules):
      module.setTargetState(swerveModuleStates[index])

    if self._targetPoseAlignmentState != State.Running:
      if chassisSpeeds.vx != 0 or chassisSpeeds.vy != 0 or chassisSpeeds.omega != 0:
        self._targetPoseAlignmentState = State.Stopped

  def _getModuleStates(self) -> tuple[SwerveModuleState, ...]:
    return tuple(module.getState() for module in self._modules)

  def _setIdleMode(self, idleMode: MotorIdleMode) -> None:
    for module in self._modules: module.setIdleMode(idleMode)
    SmartDashboard.putString("Robot/Drive/IdleMode/selected", idleMode.name)

  def lockSwerveModules(self) -> Command:
    return self.startEnd(
      lambda: self._setSwerveModulesLockPosition(Position.Locked),
      lambda: self._setSwerveModulesLockPosition(Position.Unlocked)
    ).withName("Drive:LockSwerveModules")
  
  def _setSwerveModulesLockPosition(self, position: Position) -> None:
    self._modulesLockPosition = position
    if position == Position.Locked:
      for index, module in enumerate(self._modules): 
        module.setTargetState(SwerveModuleState(0, Rotation2d.fromDegrees(45 if index in { 0, 3 } else -45)))

  def alignToTargetHeading(self, getRobotPose: Callable[[], Pose2d], getTargetPose: Callable[[], Pose3d]) -> Command:
    return cmd.startRun(
      lambda: self._initTargetHeadingAlignment(getTargetPose()),
      lambda: self._runTargetHeadingAlignment(getRobotPose())
    ).finallyDo(
      lambda end: self._endTargetHeadingAlignment()
    )

  def _initTargetHeadingAlignment(self, targetPose: Pose3d) -> None:
    self._targetHeadingAlignmentController.reset()
    self._targetPose = targetPose
    self._targetHeadingAlignmentState = State.Running

  def _runTargetHeadingAlignment(self, robotPose: Pose2d) -> None:
    self._targetHeadingAlignmentController.setSetpoint(utils.wrapAngle(utils.getTargetHeading(robotPose, self._targetPose)))
    self._targetHeadingAlignmentRotationInput = self._targetHeadingAlignmentController.calculate(robotPose.rotation().degrees()) if not self._targetHeadingAlignmentController.atSetpoint() else 0

  def _endTargetHeadingAlignment(self) -> None:
    self._targetHeadingAlignmentState = State.Stopped
    self._targetPose = None
    self._targetHeadingAlignmentRotationInput = 0

  def isAlignedToTargetHeading(self) -> bool:
    return self._targetHeadingAlignmentState == State.Running and self._targetHeadingAlignmentController.atSetpoint()

  def alignToTargetPose(self, getRobotPose: Callable[[], Pose2d], getTargetPose: Callable[[], Pose3d]) -> Command:
    return self.startRun(
      lambda: self._initTargetPoseAlignment(getRobotPose(), getTargetPose()),
      lambda: self._runTargetPoseAlignment(getRobotPose())
    ).until(
      lambda: self._targetPoseAlignmentState == State.Completed
    ).finallyDo(
      lambda end: self._endTargetPoseAlignment()
    )
  
  def _initTargetPoseAlignment(self, targetPose: Pose3d) -> None:
    self._targetPose = targetPose
    self._targetPoseAlignmentState = State.Running

  def _runTargetPoseAlignment(self, robotPose: Pose2d) -> None:
    # TODO: try applying relative/percentage speed limiters to translational velocity (rotational velocity is already being constrained in controller setup)
    self._setModuleStates(self._targetPoseAlignmentController.calculate(robotPose, self._targetPose.toPose2d(), 0, self._targetPose.toPose2d().rotation()))
    if self._targetPoseAlignmentController.atReference():
      self._targetPoseAlignmentState = State.Completed

  def _endTargetPoseAlignment(self) -> None:
    self._setModuleStates(ChassisSpeeds())
    if self._targetPoseAlignmentState != State.Completed:
      self._targetPoseAlignmentState = State.Stopped
    self._targetPose = None

  def isAlignedToTargetPose(self) -> bool:
    return self._targetPoseAlignmentState == State.Completed
  
  def reset(self) -> None:
    self.setChassisSpeeds(ChassisSpeeds())
    self._driftCorrectionState == State.Stopped
    self._targetHeadingAlignmentState == State.Stopped
    self._targetPoseAlignmentState = State.Stopped

  def _updateTelemetry(self) -> None:
    self._modulesStatesPublisher.set(self._getModuleStates())
    SmartDashboard.putBoolean("Robot/Drive/IsAlignedToTargetHeading", self.isAlignedToTargetHeading())
    SmartDashboard.putString("Robot/Drive/TargetHeadingAlignmentState", self._targetHeadingAlignmentState.name)
    SmartDashboard.putBoolean("Robot/Drive/IsAlignedToTargetPose", self.isAlignedToTargetPose())
    SmartDashboard.putString("Robot/Drive/TargetPoseAlignmentState", self._targetPoseAlignmentState.name)
    SmartDashboard.putString("Robot/Drive/Modules/LockPosition", self._modulesLockPosition.name)
