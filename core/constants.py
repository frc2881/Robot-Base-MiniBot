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

_APRILTAG_FIELD_LAYOUT = AprilTagFieldLayout(f'{ wpilib.getDeployDirectory() }/localization/2025-reefscape-andymark-filtered.json')
_PATHPLANNER_ROBOT_CONFIG = RobotConfig.fromGUISettings()

class Subsystems:
  class Drive:
    ROBOT_LENGTH: units.meters = units.inchesToMeters(19.5)
    ROBOT_WIDTH: units.meters = units.inchesToMeters(19.5)
    WHEEL_BASE: units.meters = units.inchesToMeters(9.125)
    TRACK_WIDTH: units.meters = units.inchesToMeters(9.125)

    TRANSLATION_SPEED_MAX: units.meters_per_second = 4.46
    ROTATION_SPEED_MAX: units.degrees_per_second = 720.0

    INPUT_LIMIT_DEMO: units.percent = 0.5
    INPUT_RATE_LIMIT_DEMO: units.percent = 0.5

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

    SWERVE_MODULE_CONFIGS: tuple[SwerveModuleConfig, ...] = (
      SwerveModuleConfig(SwerveModuleLocation.FrontLeft, 2, 3, -90, Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), _swerveModuleConstants),
      SwerveModuleConfig(SwerveModuleLocation.FrontRight, 4, 5, 0, Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), _swerveModuleConstants),
      SwerveModuleConfig(SwerveModuleLocation.RearLeft, 6, 7, 180, Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), _swerveModuleConstants),
      SwerveModuleConfig(SwerveModuleLocation.RearRight, 8, 9, 90, Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2), _swerveModuleConstants)
    )

    DRIVE_KINEMATICS = SwerveDrive4Kinematics(*(c.translation for c in SWERVE_MODULE_CONFIGS))

    PATHPLANNER_ROBOT_CONFIG = _PATHPLANNER_ROBOT_CONFIG
    PATHPLANNER_CONTROLLER = PPHolonomicDriveController(PIDConstants(5.0, 0, 0), PIDConstants(5.0, 0, 0))

    DRIFT_CORRECTION_CONSTANTS = DriftCorrectionConstants(
      rotationPID = PID(0.01, 0, 0), 
      rotationTolerance = Tolerance(0.5, 1.0)
    )

    TARGET_ALIGNMENT_CONSTANTS = TargetAlignmentConstants(
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
    VISION_MAX_TARGET_DISTANCE: units.meters = 4.0
    VISION_MAX_POSE_AMBIGUITY: units.percent = 0.2
    VISION_MAX_GROUND_PLANE_DELTA: units.meters = 0.25

class Sensors: 
  class Gyro:
    class NAVX2:
      COM_TYPE = AHRS.NavXComType.kMXP_SPI
  
  class Pose:
    _poseSensorConstants = PoseSensorConstants(
      aprilTagFieldLayout = _APRILTAG_FIELD_LAYOUT,
      poseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      fallbackPoseStrategy = PoseStrategy.LOWEST_AMBIGUITY
    )

    POSE_SENSOR_CONFIGS: tuple[PoseSensorConfig, ...] = (
      PoseSensorConfig(
        name = "Front",
        transform = Transform3d(Translation3d(0.149506, -0.055318, 0.271137), Rotation3d(-0.001852, -0.181301, 0.020370)), 
        stream = "http://10.28.81.6:1182/?action=stream", 
        constants = _poseSensorConstants
      ),
    )

class Cameras:
  DRIVER_STREAM = "http://10.28.81.6:1182/?action=stream"

class Controllers:
  DRIVER_CONTROLLER_PORT: int = 0
  OPERATOR_CONTROLLER_PORT: int = 1
  INPUT_DEADBAND: units.percent = 0.1

class Game:
  class Commands:
    AUTO_TARGET_ALIGNMENT_TIMEOUT: units.seconds = 1.5

  class Field:
    APRILTAG_FIELD_LAYOUT = _APRILTAG_FIELD_LAYOUT
    LENGTH = _APRILTAG_FIELD_LAYOUT.getFieldLength()
    WIDTH = _APRILTAG_FIELD_LAYOUT.getFieldWidth()
    BOUNDS = (Translation2d(0, 0), Translation2d(LENGTH, WIDTH))

    class Targets:
      TARGETS: dict[Alliance, dict[int, Target]] = {
        Alliance.Red: {
          utils.getTargetHash(_APRILTAG_FIELD_LAYOUT.getTagPose(1).toPose2d()): Target(TargetType.CoralStation, _APRILTAG_FIELD_LAYOUT.getTagPose(1)),
          utils.getTargetHash(_APRILTAG_FIELD_LAYOUT.getTagPose(2).toPose2d()): Target(TargetType.CoralStation, _APRILTAG_FIELD_LAYOUT.getTagPose(2)),
          utils.getTargetHash(_APRILTAG_FIELD_LAYOUT.getTagPose(6).toPose2d()): Target(TargetType.Reef, _APRILTAG_FIELD_LAYOUT.getTagPose(6)),
          utils.getTargetHash(_APRILTAG_FIELD_LAYOUT.getTagPose(7).toPose2d()): Target(TargetType.Reef, _APRILTAG_FIELD_LAYOUT.getTagPose(7)),
          utils.getTargetHash(_APRILTAG_FIELD_LAYOUT.getTagPose(8).toPose2d()): Target(TargetType.Reef, _APRILTAG_FIELD_LAYOUT.getTagPose(8)),
          utils.getTargetHash(_APRILTAG_FIELD_LAYOUT.getTagPose(9).toPose2d()): Target(TargetType.Reef, _APRILTAG_FIELD_LAYOUT.getTagPose(9)),
          utils.getTargetHash(_APRILTAG_FIELD_LAYOUT.getTagPose(10).toPose2d()): Target(TargetType.Reef, _APRILTAG_FIELD_LAYOUT.getTagPose(10)),
          utils.getTargetHash(_APRILTAG_FIELD_LAYOUT.getTagPose(11).toPose2d()): Target(TargetType.Reef, _APRILTAG_FIELD_LAYOUT.getTagPose(11))
        },
        Alliance.Blue: {
          utils.getTargetHash(_APRILTAG_FIELD_LAYOUT.getTagPose(12).toPose2d()): Target(TargetType.CoralStation, _APRILTAG_FIELD_LAYOUT.getTagPose(12)),
          utils.getTargetHash(_APRILTAG_FIELD_LAYOUT.getTagPose(13).toPose2d()): Target(TargetType.CoralStation, _APRILTAG_FIELD_LAYOUT.getTagPose(13)),
          utils.getTargetHash(_APRILTAG_FIELD_LAYOUT.getTagPose(17).toPose2d()): Target(TargetType.Reef, _APRILTAG_FIELD_LAYOUT.getTagPose(17)),
          utils.getTargetHash(_APRILTAG_FIELD_LAYOUT.getTagPose(18).toPose2d()): Target(TargetType.Reef, _APRILTAG_FIELD_LAYOUT.getTagPose(18)),
          utils.getTargetHash(_APRILTAG_FIELD_LAYOUT.getTagPose(19).toPose2d()): Target(TargetType.Reef, _APRILTAG_FIELD_LAYOUT.getTagPose(19)),
          utils.getTargetHash(_APRILTAG_FIELD_LAYOUT.getTagPose(20).toPose2d()): Target(TargetType.Reef, _APRILTAG_FIELD_LAYOUT.getTagPose(20)),
          utils.getTargetHash(_APRILTAG_FIELD_LAYOUT.getTagPose(21).toPose2d()): Target(TargetType.Reef, _APRILTAG_FIELD_LAYOUT.getTagPose(21)),
          utils.getTargetHash(_APRILTAG_FIELD_LAYOUT.getTagPose(22).toPose2d()): Target(TargetType.Reef, _APRILTAG_FIELD_LAYOUT.getTagPose(22))
        }
      }

      TARGET_ALIGNMENT_TRANSFORMS: dict[TargetType, dict[TargetAlignmentLocation, Transform3d]] = {
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
