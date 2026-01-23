from typing import Callable
from commands2 import Subsystem, Command, cmd
from wpilib import SmartDashboard, SendableChooser
from wpimath import units
from wpimath.controller import PIDController, ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Rotation2d, Pose2d, Pose3d, Transform2d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition, SwerveModuleState, SwerveDrive4Kinematics
from ntcore import NetworkTableInstance
from pathplannerlib.util import DriveFeedforwards
from lib import logger, utils
from lib.classes import State, Position, MotorIdleMode, SpeedMode, DriveOrientation, TargetAlignmentMode
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

    self._inputXFilter = SlewRateLimiter(self._constants.INPUT_RATE_LIMIT_DEMO)
    self._inputYFilter = SlewRateLimiter(self._constants.INPUT_RATE_LIMIT_DEMO)
    self._inputRotationFilter = SlewRateLimiter(self._constants.INPUT_RATE_LIMIT_DEMO)

    self._driftCorrectionState = State.Stopped
    self._driftCorrectionController = PIDController(*self._constants.DRIFT_CORRECTION_CONSTANTS.rotationPID)
    self._driftCorrectionController.setTolerance(self._constants.DRIFT_CORRECTION_CONSTANTS.rotationPositionTolerance)
    self._driftCorrectionController.enableContinuousInput(-180.0, 180.0)

    self._targetLockState = State.Stopped
    self._targetLockController = PIDController(*self._constants.TARGET_LOCK_CONSTANTS.rotationPID)
    self._targetLockController.setTolerance(self._constants.TARGET_LOCK_CONSTANTS.rotationPositionTolerance)
    self._targetLockController.enableContinuousInput(-180.0, 180.0)
    self._targetLockInputRotationOverride: units.percent = 0

    self._targetAlignmentState = State.Stopped
    self._targetAlignmentTranslationXController = ProfiledPIDController(
      *self._constants.TARGET_ALIGNMENT_CONSTANTS.translationPID, 
      TrapezoidProfile.Constraints(self._constants.TARGET_ALIGNMENT_CONSTANTS.translationMaxVelocity, self._constants.TARGET_ALIGNMENT_CONSTANTS.translationMaxAcceleration)
    )
    self._targetAlignmentTranslationXController.setTolerance(self._constants.TARGET_ALIGNMENT_CONSTANTS.translationPositionTolerance, self._constants.TARGET_ALIGNMENT_CONSTANTS.translationVelocityTolerance)
    self._targetAlignmentTranslationYController = ProfiledPIDController(
      *self._constants.TARGET_ALIGNMENT_CONSTANTS.translationPID, 
      TrapezoidProfile.Constraints(self._constants.TARGET_ALIGNMENT_CONSTANTS.translationMaxVelocity, self._constants.TARGET_ALIGNMENT_CONSTANTS.translationMaxAcceleration)
    )
    self._targetAlignmentTranslationYController.setTolerance(self._constants.TARGET_ALIGNMENT_CONSTANTS.translationPositionTolerance, self._constants.TARGET_ALIGNMENT_CONSTANTS.translationVelocityTolerance)
    self._targetAlignmentRotationController = ProfiledPIDController(
      *self._constants.TARGET_ALIGNMENT_CONSTANTS.rotationPID, 
      TrapezoidProfile.Constraints(self._constants.TARGET_ALIGNMENT_CONSTANTS.rotationMaxVelocity, self._constants.TARGET_ALIGNMENT_CONSTANTS.rotationMaxAcceleration)
    )
    self._targetAlignmentRotationController.setTolerance(self._constants.TARGET_ALIGNMENT_CONSTANTS.rotationPositionTolerance, self._constants.TARGET_ALIGNMENT_CONSTANTS.rotationVelocityTolerance)
    self._targetAlignmentRotationController.enableContinuousInput(-180.0, 180.0)

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

    self._swerveModulesLockPosition = Position.Unlocked

  def periodic(self) -> None:
    self._updateTelemetry()

  def drive(self, getInputX: Callable[[], units.percent], getInputY: Callable[[], units.percent], getInputRotation: Callable[[], units.percent]) -> Command:
    return self.run(
      lambda: self._drive(getInputX(), getInputY(), getInputRotation())
    ).onlyIf(
      lambda: self._swerveModulesLockPosition == Position.Unlocked
    ).withName("Drive:Drive")

  def _drive(self, inputX: units.percent, inputY: units.percent, inputRotation: units.percent) -> None:
    if self._targetLockState == State.Running:
      inputRotation = self._targetLockInputRotationOverride
    else:
      if self._driftCorrection == State.Enabled:
        isTranslating: bool = inputX != 0 or inputY != 0
        isRotating: bool = inputRotation != 0
        if isTranslating and not isRotating and not self._driftCorrectionState == State.Running:
          self._driftCorrectionState = State.Running
          self._driftCorrectionController.reset()
          self._driftCorrectionController.setSetpoint(self._getGyroHeading())
        elif isRotating or not isTranslating:
          self._driftCorrectionState = State.Stopped
        if self._driftCorrectionState == State.Running:
          inputRotation = self._driftCorrectionController.calculate(self._getGyroHeading())
          if self._driftCorrectionController.atSetpoint():
            inputRotation = 0

    if self._speedMode == SpeedMode.Demo:
      inputX = self._inputXFilter.calculate(inputX * self._constants.INPUT_LIMIT_DEMO) if inputX != 0 else 0
      inputY = self._inputYFilter.calculate(inputY * self._constants.INPUT_LIMIT_DEMO) if inputY != 0 else 0
      inputRotation = self._inputRotationFilter.calculate(inputRotation * self._constants.INPUT_LIMIT_DEMO) if inputRotation != 0 else 0

    speedX: units.meters_per_second = inputX * self._constants.TRANSLATION_SPEED_MAX
    speedY: units.meters_per_second = inputY * self._constants.TRANSLATION_SPEED_MAX
    speedRotation: units.degrees_per_second = inputRotation * self._constants.ROTATION_SPEED_MAX
    
    self.setChassisSpeeds(
      ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, units.degreesToRadians(speedRotation), Rotation2d.fromDegrees(self._getGyroHeading()))
      if self._orientation == DriveOrientation.Field else
      ChassisSpeeds(speedX, speedY, units.degreesToRadians(speedRotation))
    )

  def setChassisSpeeds(self, chassisSpeeds: ChassisSpeeds, driveFeedforwards: DriveFeedforwards = None) -> None:
    self._setModuleStates(chassisSpeeds)

  def getChassisSpeeds(self) -> ChassisSpeeds:
    return self._constants.DRIVE_KINEMATICS.toChassisSpeeds(self._getModuleStates())

  def getModulePositions(self) -> tuple[SwerveModulePosition, ...]:
    return tuple(m.getPosition() for m in self._modules)

  def _setModuleStates(self, chassisSpeeds: ChassisSpeeds) -> None: 
    swerveModuleStates = SwerveDrive4Kinematics.desaturateWheelSpeeds(
      self._constants.DRIVE_KINEMATICS.toSwerveModuleStates(
        ChassisSpeeds.discretize(
          self._constants.DRIVE_KINEMATICS.toChassisSpeeds(
            SwerveDrive4Kinematics.desaturateWheelSpeeds(
              self._constants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds), 
              self._constants.TRANSLATION_SPEED_MAX
            )
          ), 0.02
        )
      ), self._constants.TRANSLATION_SPEED_MAX
    )
    for i, m in enumerate(self._modules):
      m.setTargetState(swerveModuleStates[i])

    if self._targetAlignmentState != State.Running:
      if chassisSpeeds.vx != 0 or chassisSpeeds.vy != 0 or chassisSpeeds.omega != 0:
        self._targetAlignmentState = State.Stopped

  def _getModuleStates(self) -> tuple[SwerveModuleState, ...]:
    return tuple(m.getState() for m in self._modules)

  def _setIdleMode(self, idleMode: MotorIdleMode) -> None:
    for m in self._modules: m.setIdleMode(idleMode)
    SmartDashboard.putString("Robot/Drive/IdleMode/selected", idleMode.name)

  def lockSwerveModules(self) -> Command:
    return self.startEnd(
      lambda: self._setSwerveModuleLockPosition(Position.Locked),
      lambda: self._setSwerveModuleLockPosition(Position.Unlocked)
    ).withName("Drive:LockSwerveModules")
  
  def _setSwerveModuleLockPosition(self, position: Position) -> None:
    self._swerveModulesLockPosition = position
    if position == Position.Locked:
      for i, m in enumerate(self._modules): 
        m.setTargetState(SwerveModuleState(0, Rotation2d.fromDegrees(45 if i in { 0, 3 } else -45)))

  def lockToTarget(self, getRobotPose: Callable[[], Pose2d], getTargetPose: Callable[[], Pose3d]) -> Command:
    return cmd.startRun(
      lambda: self._initTargetLock(getRobotPose(), getTargetPose()),
      lambda: self._runTargetLock(getRobotPose(), getTargetPose())
    ).finallyDo(
      lambda end: self._endTargetLock()
    ).withName("Drive:LockToTarget")

  def _initTargetLock(self, robotPose: Pose2d, targetPose: Pose3d) -> None:
    self._targetLockState = State.Running
    self._targetLockController.reset()

  def _runTargetLock(self, robotPose: Pose2d, targetPose: Pose3d) -> None:
    self._targetLockController.setSetpoint(utils.wrapAngle(utils.getTargetHeading(robotPose, targetPose)))
    self._targetLockInputRotationOverride = self._targetLockController.calculate(robotPose.rotation().degrees()) if not self._targetLockController.atSetpoint() else 0

  def _endTargetLock(self) -> None:
    self._targetLockState = State.Stopped
    self._targetLockInputRotationOverride = 0

  def isLockedToTarget(self) -> bool:
    return self._targetLockState == State.Running

  def alignToTarget(self, getRobotPose: Callable[[], Pose2d], getTargetPose: Callable[[], Pose3d], targetAlignmentMode: TargetAlignmentMode) -> Command:
    return self.startRun(
      lambda: self._initTargetAlignment(getRobotPose(), getTargetPose(), targetAlignmentMode),
      lambda: self._runTargetAlignment(getRobotPose(), getTargetPose(), targetAlignmentMode)
    ).until(
      lambda: self._targetAlignmentState == State.Completed
    ).finallyDo(
      lambda end: self._endTargetAlignment()
    ).withName("Drive:AlignToTarget")
  
  def _initTargetAlignment(self, robotPose: Pose2d, targetPose: Pose3d, targetAlignmentMode: TargetAlignmentMode) -> None:
    self._targetAlignmentState = State.Running
    self._targetAlignmentTranslationXController.reset(0)
    self._targetAlignmentTranslationXController.setGoal(0)
    self._targetAlignmentTranslationYController.reset(0)
    self._targetAlignmentTranslationYController.setGoal(0)
    self._targetAlignmentRotationController.reset(robotPose.rotation().degrees())
    self._targetAlignmentRotationController.setGoal(
      targetPose.toPose2d().rotation().degrees()
      if targetAlignmentMode == TargetAlignmentMode.Pose else
      utils.wrapAngle(utils.getTargetHeading(robotPose, targetPose))
    )

  def _runTargetAlignment(self, robotPose: Pose2d, targetPose: Pose3d, targetAlignmentMode: TargetAlignmentMode) -> None:
    if (
      not self._targetAlignmentTranslationXController.atGoal() or
      not self._targetAlignmentTranslationYController.atGoal() or
      not self._targetAlignmentRotationController.atGoal()
    ):
      targetTranslation = targetPose.toPose2d() - robotPose if targetAlignmentMode == TargetAlignmentMode.Pose else Transform2d()
      vx = utils.clampValue(
        self._targetAlignmentTranslationXController.calculate(targetTranslation.X()),
        -self._constants.TARGET_ALIGNMENT_CONSTANTS.translationMaxVelocity,
        self._constants.TARGET_ALIGNMENT_CONSTANTS.translationMaxVelocity
      )
      vy = utils.clampValue(
        self._targetAlignmentTranslationYController.calculate(targetTranslation.Y()),
        -self._constants.TARGET_ALIGNMENT_CONSTANTS.translationMaxVelocity,
        self._constants.TARGET_ALIGNMENT_CONSTANTS.translationMaxVelocity
      )
      vt = utils.clampValue(
        self._targetAlignmentRotationController.calculate(robotPose.rotation().degrees()),
        -self._constants.TARGET_ALIGNMENT_CONSTANTS.rotationMaxVelocity,
        self._constants.TARGET_ALIGNMENT_CONSTANTS.rotationMaxVelocity
      )
      self._setModuleStates(ChassisSpeeds(-vx, -vy, units.degreesToRadians(vt)))
    else:
      self._targetAlignmentState = State.Completed

  def _endTargetAlignment(self) -> None:
    self._setModuleStates(ChassisSpeeds())
    if self._targetAlignmentState != State.Completed:
      self._targetAlignmentState = State.Stopped

  def isAlignedToTarget(self) -> bool:
    return self._targetAlignmentState == State.Completed
  
  def reset(self) -> None:
    self.setChassisSpeeds(ChassisSpeeds())
    self._driftCorrectionState == State.Stopped
    self._targetLockState == State.Stopped
    self._targetAlignmentState = State.Stopped

  def _updateTelemetry(self) -> None:
    self._modulesStatesPublisher.set(self._getModuleStates())
    SmartDashboard.putBoolean("Robot/Drive/IsLockedToTarget", self.isLockedToTarget())
    SmartDashboard.putBoolean("Robot/Drive/IsAlignedToTarget", self.isAlignedToTarget())
    SmartDashboard.putString("Robot/Drive/TargetAlignmentState", self._targetAlignmentState.name)
    SmartDashboard.putString("Robot/Drive/Modules/LockPosition", self._swerveModulesLockPosition.name)
