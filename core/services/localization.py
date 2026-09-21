from typing import TYPE_CHECKING, Callable, Optional
from wpilib import SmartDashboard, Timer
from wpimath import units
from wpimath.geometry import Pose2d, Rotation2d, Rectangle2d
if TYPE_CHECKING: from wpimath.kinematics import SwerveModulePosition
from wpimath.estimator import SwerveDrive4PoseEstimator
from ntcore import NetworkTableInstance
from lib import logger, utils
from lib.classes import Alliance, RobotState, PoseSensorResult, PoseSensorResultType, Value
if TYPE_CHECKING: from lib.sensors.pose import PoseSensor
from core.classes import Zone
import core.constants as constants

class Localization():
  def __init__(
      self,
      getGyroHeading: Callable[[], units.degrees],
      getDriveModulePositions: Callable[[], tuple[SwerveModulePosition, SwerveModulePosition, SwerveModulePosition, SwerveModulePosition]],
      poseSensors: tuple[PoseSensor, ...]
    ) -> None:
    self._constants = constants.Services.Localization
    self._getGyroHeading = getGyroHeading
    self._getDriveModulePositions = getDriveModulePositions
    self._poseSensors = poseSensors

    self._poseEstimator = SwerveDrive4PoseEstimator(
      constants.Subsystems.Drive.DRIVE_KINEMATICS,
      Rotation2d.fromDegrees(self._getGyroHeading()),
      self._getDriveModulePositions(),
      Pose2d()
    )
    
    self._hasValidPoseSensorResult: bool = False
    self._validPoseSensorResultTimer = Timer()
    
    self._robotPosePublisher = NetworkTableInstance.getDefault().getStructTopic("/SmartDashboard/Robot/Localization/Pose", Pose2d).publish()

    self._alliance: Optional[Alliance] = None
    self._zones: dict[Zone, Rectangle2d] = {}
    self._robotZone: Optional[Zone] = None

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateRobotPose()
    self._updateZones()
    self._updateRobotZone()
    self._updateTelemetry()

  def _updateRobotPose(self) -> None:
    self._poseEstimator.update(Rotation2d.fromDegrees(self._getGyroHeading()), self._getDriveModulePositions())
    hasValidPoseSensorResult = False
    for poseSensor in self._poseSensors:
      poseSensorResult = poseSensor.getLatestResult()
      if poseSensorResult is not None:
        if self._isResultValid(poseSensorResult):
          self._poseEstimator.addVisionMeasurement(
            poseSensorResult.estimatedPose.toPose2d(), 
            poseSensorResult.timestamp,
            self._getStandardDeviations(poseSensorResult)
          )
          hasValidPoseSensorResult = True
    if hasValidPoseSensorResult:
      self._hasValidPoseSensorResult = True
      self._validPoseSensorResultTimer.restart()
    else:
      if self._hasValidPoseSensorResult and self._validPoseSensorResultTimer.hasElapsed(self._constants.VALID_POSE_SENSOR_RESULT_TIMEOUT):
        self._hasValidPoseSensorResult = False

  def _isResultValid(self, poseSensorResult: PoseSensorResult) -> bool:
    return (         
      utils.isPoseWithinBounds(poseSensorResult.estimatedPose.toPose2d(), constants.Game.Field.BOUNDS) 
      and
      poseSensorResult.bestTargetDistance <= self._constants.MAX_TARGET_DISTANCE 
      and
      (
        poseSensorResult.resultType == PoseSensorResultType.SINGLE_TAG or 
        poseSensorResult.bestTargetReprojectionError <= self._constants.MAX_TARGET_REPROJECTION_ERROR
      )
      and
      (
        poseSensorResult.resultType == PoseSensorResultType.MULTI_TAG or
        poseSensorResult.bestTargetAmbiguity <= self._constants.MAX_TARGET_AMBIGUITY
      )
      and
      (
        utils.getRobotState() == RobotState.Disabled or 
        poseSensorResult.resultType == PoseSensorResultType.MULTI_TAG or
        utils.getTargetDistance(poseSensorResult.estimatedPose, self._poseEstimator.getEstimatedPosition()) <= self._constants.MAX_POSE_CHANGE
      )
    )
  
  def _getStandardDeviations(self, poseSensorResult: PoseSensorResult) -> tuple[float, float, float]:
    stdDevXY = self._constants.STDDEV_XY_COEFF * poseSensorResult.bestTargetDistance
    stdDevZ = self._constants.STDDEV_Z_COEFF * poseSensorResult.bestTargetDistance
    if poseSensorResult.resultType == PoseSensorResultType.MULTI_TAG:
      stdDevXY *= self._constants.STDDEV_TARGET_REPROJECTION_ERROR_SCALE_FACTOR * poseSensorResult.bestTargetReprojectionError
      stdDevZ *= self._constants.STDDEV_TARGET_REPROJECTION_ERROR_SCALE_FACTOR * poseSensorResult.bestTargetReprojectionError
    else: 
      stdDevXY *= self._constants.STDDEV_TARGET_AMBIGUITY_SCALE_FACTOR * (poseSensorResult.bestTargetAmbiguity + 0.01)
      stdDevZ = Value.max
    return (stdDevXY, stdDevXY, stdDevZ)

  def getRobotPose(self) -> Pose2d:
    return self._poseEstimator.getEstimatedPosition()

  def resetRobotPose(self, pose: Pose2d) -> None:
    self._poseEstimator.resetPose(pose)
  
  def hasValidPoseSensorResult(self) -> bool:
    return self._hasValidPoseSensorResult

  def _updateZones(self) -> None:
    if utils.getAlliance() != self._alliance:
      self._alliance = utils.getAlliance()
      self._zones = constants.Game.Field.ZONES[self._alliance]

  def _updateRobotZone(self) -> None:
    for zone in self._zones:
      if utils.isPoseWithinBounds(self.getRobotPose(), self._zones[zone]):
        self._robotZone = zone
        return
    self._robotZone = None

  def getRobotZone(self) -> Optional[Zone]:
    return self._robotZone

  def _updateTelemetry(self) -> None:
    self._robotPosePublisher.set(self.getRobotPose())
    SmartDashboard.putBoolean("Robot/Localization/HasValidPoseSensorResult", self.hasValidPoseSensorResult())
    SmartDashboard.putString("Robot/Localization/Zone", self._robotZone.name if self._robotZone is not None else "")
