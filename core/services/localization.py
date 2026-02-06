from typing import TYPE_CHECKING, Callable
from wpilib import SmartDashboard, Timer
from wpimath import units
from wpimath.geometry import Pose2d, Pose3d, Rotation2d, Transform2d
if TYPE_CHECKING: from wpimath.kinematics import SwerveModulePosition
from wpimath.estimator import SwerveDrive4PoseEstimator
from ntcore import NetworkTableInstance
from lib import logger, utils
from lib.classes import RobotState
if TYPE_CHECKING: from lib.sensors.pose import PoseSensor
if TYPE_CHECKING: from lib.sensors.object import ObjectSensor
from core.classes import Target
import core.constants as constants

class Localization():
  def __init__(
      self,
      getGyroHeading: Callable[[], units.degrees],
      getDriveModulePositions: Callable[[], tuple[SwerveModulePosition, ...]],
      poseSensors: tuple[PoseSensor, ...],
      objectSensor: ObjectSensor
    ) -> None:
    super().__init__()
    self._getGyroHeading = getGyroHeading
    self._getDriveModulePositions = getDriveModulePositions
    self._poseSensors = poseSensors
    self._objectSensor = objectSensor

    self._poseEstimator = SwerveDrive4PoseEstimator(
      constants.Subsystems.Drive.DRIVE_KINEMATICS,
      Rotation2d.fromDegrees(self._getGyroHeading()),
      self._getDriveModulePositions(),
      Pose2d()
    )
    
    self._alliance = None
    self._targets: dict[Target, Pose3d] = {}
    self._robotPose = Pose2d()
    self._objectsTransform = Transform2d()
    self._objectsCount: int = 0
    self._hasValidVisionTarget: bool = False
    self._validVisionTargetBufferTimer = Timer()
    
    self._robotPosePublisher = NetworkTableInstance.getDefault().getStructTopic("/SmartDashboard/Robot/Localization/Pose", Pose2d).publish()

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTargets()
    self._updateRobotPose()
    self._updateObjects()
    self._updateTelemetry()

  def _updateTargets(self) -> None:
    if utils.getAlliance() != self._alliance:
      self._alliance = utils.getAlliance()
      self._targets = constants.Game.Field.Targets.TARGETS[self._alliance]

  def _updateRobotPose(self) -> None:
    self._poseEstimator.update(Rotation2d.fromDegrees(self._getGyroHeading()), self._getDriveModulePositions())
    hasValidVisionTarget = False
    for poseSensor in self._poseSensors:
      estimatedRobotPose = poseSensor.getEstimatedRobotPose()
      if estimatedRobotPose is not None:
        estimatedPose = estimatedRobotPose.estimatedPose.toPose2d()
        if utils.isPoseInBounds(estimatedPose, constants.Game.Field.BOUNDS):
          poseAmbiguity = estimatedRobotPose.targetsUsed[0].getPoseAmbiguity()
          if utils.isValueInRange(poseAmbiguity, -1, constants.Services.Localization.VISION_MAX_POSE_AMBIGUITY):
            hasValidVisionTarget = True
            if (
              utils.getRobotState() == RobotState.Disabled or 
              utils.isValueInRange(estimatedPose.translation().distance(self._poseEstimator.getEstimatedPosition().translation()), 0, constants.Services.Localization.VISION_MAX_ESTIMATED_POSE_DELTA)
            ):
              self._poseEstimator.addVisionMeasurement(
                estimatedPose, 
                estimatedRobotPose.timestampSeconds,
                constants.Services.Localization.VISION_ESTIMATE_MULTI_TAG_STANDARD_DEVIATIONS
                if poseAmbiguity == -1 else
                constants.Services.Localization.VISION_ESTIMATE_SINGLE_TAG_STANDARD_DEVIATIONS
              )     
    self._robotPose = self._poseEstimator.getEstimatedPosition()
    if hasValidVisionTarget:
      self._hasValidVisionTarget = True
      self._validVisionTargetBufferTimer.restart()
    else:
      if self._hasValidVisionTarget and self._validVisionTargetBufferTimer.hasElapsed(0.1):
        self._hasValidVisionTarget = False

  def hasValidVisionTarget(self) -> bool:
    return self._hasValidVisionTarget

  def getRobotPose(self) -> Pose2d:
    return self._robotPose

  def resetRobotPose(self, pose: Pose2d) -> None:
    self._poseEstimator.resetPose(pose)

  def getTargetPose(self, target: Target) -> Pose3d:
    targetPose = self._targets.get(target)
    return targetPose if targetPose is not None else Pose3d(self._robotPose)

  def _updateObjects(self) -> None:
    objects = self._objectSensor.getObjects()
    if objects is not None:
      self._objectsTransform = objects.transform
      self._objectsCount = objects.count
    else:
      self._objectsTransform = Transform2d()
      self._objectsCount = 0

  def hasObjects(self) -> bool:
    return self._objectsCount > 0
  
  def getObjectsCount(self) -> int:
    return self._objectsCount

  def getObjectsPose(self) -> Pose2d:
    return self._robotPose.transformBy(self._objectsTransform)

  def _updateTelemetry(self) -> None:
    self._robotPosePublisher.set(self.getRobotPose())
    SmartDashboard.putBoolean("Robot/Localization/HasValidVisionTarget", self.hasValidVisionTarget())
    SmartDashboard.putBoolean("Robot/Localization/HasObjects", self.hasObjects())
    SmartDashboard.putNumber("Robot/Localization/Objects/Heading", self._objectsTransform.rotation().degrees())
    SmartDashboard.putNumber("Robot/Localization/Objects/Distance", self._objectsTransform.translation().norm())
    SmartDashboard.putNumber("Robot/Localization/Objects/Count", self.getObjectsCount())
