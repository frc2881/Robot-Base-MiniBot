import wpilib
from wpimath import units
from wpimath.geometry import Transform3d, Translation3d, Rotation3d, Translation2d, Rotation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from robotpy_apriltag import AprilTagFieldLayout
from navx import AHRS
from rev import SparkLowLevel
from photonlibpy.photonPoseEstimator import PoseStrategy
from pathplannerlib.config import RobotConfig
from pathplannerlib.controller import PPHolonomicDriveController, PIDConstants
from lib import logger, utils
from lib.classes import (
  Alliance, 
  PID, 
  Tolerance,
  SwerveModuleConstants, 
  SwerveModuleConfig, 
  SwerveModuleLocation, 
  DriftCorrectionConstants, 
  TargetAlignmentConstants,
  PoseSensorConstants,
  PoseSensorConfig
)
from core.classes import (
  Target, 
  TargetType, 
  TargetAlignmentLocation
)

APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout(f'{ wpilib.getDeployDirectory() }/localization/2025-reefscape-andymark-filtered.json')
PATHPLANNER_ROBOT_CONFIG = RobotConfig.fromGUISettings()

class Subsystems:
  class Drive:
    kRobotLength: units.meters = units.inchesToMeters(19.5)
    kRobotWidth: units.meters = units.inchesToMeters(19.5)
    kTrackWidth: units.meters = units.inchesToMeters(9.125)
    kWheelBase: units.meters = units.inchesToMeters(9.125)

    kTranslationSpeedMax: units.meters_per_second = 4.46
    kRotationSpeedMax: units.degrees_per_second = 720.0

    kInputLimitDemo: units.percent = 0.5
    kInputRateLimitDemo: units.percent = 0.5

    _swerveModuleConstants = SwerveModuleConstants(
      wheelDiameter = units.inchesToMeters(3.0),
      wheelBevelGearTeeth = 45,
      wheelSpurGearTeeth = 22,
      wheelBevelPinionTeeth = 15,
      drivingMotorPinionTeeth = 13,
      drivingMotorFreeSpeed = 5676,
      drivingMotorControllerType = SparkLowLevel.SparkModel.kSparkMax,
      drivingMotorType = SparkLowLevel.MotorType.kBrushless,
      drivingMotorCurrentLimit = 80,
      drivingMotorPID = PID(0.04, 0, 0),
      turningMotorCurrentLimit = 20,
      turningMotorPID = PID(1.0, 0, 0)
    )

    kSwerveModuleConfigs: tuple[SwerveModuleConfig, ...] = (
      SwerveModuleConfig(SwerveModuleLocation.FrontLeft, 2, 3, -90, Translation2d(kWheelBase / 2, kTrackWidth / 2), _swerveModuleConstants),
      SwerveModuleConfig(SwerveModuleLocation.FrontRight, 4, 5, 0, Translation2d(kWheelBase / 2, -kTrackWidth / 2), _swerveModuleConstants),
      SwerveModuleConfig(SwerveModuleLocation.RearLeft, 6, 7, 180, Translation2d(-kWheelBase / 2, kTrackWidth / 2), _swerveModuleConstants),
      SwerveModuleConfig(SwerveModuleLocation.RearRight, 8, 9, 90, Translation2d(-kWheelBase / 2, -kTrackWidth / 2), _swerveModuleConstants)
    )

    kDriveKinematics = SwerveDrive4Kinematics(*(c.translation for c in kSwerveModuleConfigs))

    kPathPlannerRobotConfig = PATHPLANNER_ROBOT_CONFIG
    kPathPlannerController = PPHolonomicDriveController(PIDConstants(5.0, 0, 0), PIDConstants(5.0, 0, 0))

    kDriftCorrectionConstants = DriftCorrectionConstants(
      rotationPID = PID(0.01, 0, 0), 
      rotationTolerance = Tolerance(0.5, 1.0)
    )

    kTargetAlignmentConstants = TargetAlignmentConstants(
      translationPID = PID(5.0, 0, 0),
      translationMaxVelocity = 1.4,
      translationMaxAcceleration = 1.0,
      translationTolerance = Tolerance(0.05, 0.1),
      rotationPID = PID(5.0, 0, 0),
      rotationMaxVelocity = 360.0,
      rotationMaxAcceleration = 180.0,
      rotationTolerance = Tolerance(0.5, 1.0),
      rotationHeadingModeOffset = 0,
      rotationTranslationModeOffset = 180.0
    )

class Services:
  class Localization:
    kVisionMaxTargetDistance: units.meters = 4.0
    kVisionMaxPoseAmbiguity: units.percent = 0.2
    kRobotPoseMaxGroundPlaneDelta: units.meters = 0.25

class Sensors: 
  class Gyro:
    class NAVX2:
      kComType = AHRS.NavXComType.kMXP_SPI
  
  class Pose:
    _poseSensorConstants = PoseSensorConstants(
      aprilTagFieldLayout = APRIL_TAG_FIELD_LAYOUT,
      poseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      fallbackPoseStrategy = PoseStrategy.LOWEST_AMBIGUITY
    )

    kPoseSensorConfigs: tuple[PoseSensorConfig, ...] = (
      PoseSensorConfig(
        name = "Front",
        transform = Transform3d(Translation3d(0.149506, -0.055318, 0.271137), Rotation3d(-0.001852, -0.181301, 0.020370)), 
        stream = "http://10.28.81.6:1182/?action=stream", 
        constants = _poseSensorConstants
      ),
    )

class Cameras:
  kDriverStream = "http://10.28.81.6:1182/?action=stream"

class Controllers:
  kDriverControllerPort: int = 0
  kOperatorControllerPort: int = 1
  kInputDeadband: units.percent = 0.1

class Game:
  class Commands:
    kAutoTargetAlignmentTimeout: units.seconds = 1.5

  class Field:
    kAprilTagFieldLayout = APRIL_TAG_FIELD_LAYOUT
    kLength = APRIL_TAG_FIELD_LAYOUT.getFieldLength()
    kWidth = APRIL_TAG_FIELD_LAYOUT.getFieldWidth()
    kBounds = (Translation2d(0, 0), Translation2d(kLength, kWidth))

    class Targets:
      kTargets: dict[Alliance, dict[int, Target]] = {
        Alliance.Red: {
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(1).toPose2d()): Target(TargetType.CoralStation, APRIL_TAG_FIELD_LAYOUT.getTagPose(1)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(2).toPose2d()): Target(TargetType.CoralStation, APRIL_TAG_FIELD_LAYOUT.getTagPose(2)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(6).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(6)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(7).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(7)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(8).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(8)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(9).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(9)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(10).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(10)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(11).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(11))
        },
        Alliance.Blue: {
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(12).toPose2d()): Target(TargetType.CoralStation, APRIL_TAG_FIELD_LAYOUT.getTagPose(12)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(13).toPose2d()): Target(TargetType.CoralStation, APRIL_TAG_FIELD_LAYOUT.getTagPose(13)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(17).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(17)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(18).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(18)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(19).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(19)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(20).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(20)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(21).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(21)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(22).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(22))
        }
      }

      kTargetAlignmentTransforms: dict[TargetType, dict[TargetAlignmentLocation, Transform3d]] = {
        TargetType.Reef: {
          TargetAlignmentLocation.Center: Transform3d(units.inchesToMeters(27.0), 0, 0, Rotation3d()),
          TargetAlignmentLocation.Left: Transform3d(units.inchesToMeters(3.0), units.inchesToMeters(-8.0), 0, Rotation3d(Rotation2d.fromDegrees(0.0))),
          TargetAlignmentLocation.Right: Transform3d(units.inchesToMeters(3.0), units.inchesToMeters(8.0), 0, Rotation3d(Rotation2d.fromDegrees(0.0))) 
        },
        TargetType.CoralStation: {
          TargetAlignmentLocation.Center: Transform3d(units.inchesToMeters(20.0), units.inchesToMeters(0.0), 0, Rotation3d(Rotation2d.fromDegrees(0.0))),
          TargetAlignmentLocation.Left: Transform3d(units.inchesToMeters(20.0), units.inchesToMeters(-24.0), 0, Rotation3d(Rotation2d.fromDegrees(0.0))),
          TargetAlignmentLocation.Right: Transform3d(units.inchesToMeters(20.0), units.inchesToMeters(24.0), 0, Rotation3d(Rotation2d.fromDegrees(0.0)))
        }
      }                                                                                                                                           
