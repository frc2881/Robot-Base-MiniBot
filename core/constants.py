import wpilib
from wpimath import units
from wpimath.geometry import Pose2d, Pose3d, Rotation3d, Translation2d, Rotation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from robotpy_apriltag import AprilTagFieldLayout
from navx import AHRS
from rev import SparkLowLevel, AbsoluteEncoderConfig
from pathplannerlib.config import RobotConfig
from pathplannerlib.controller import PPHolonomicDriveController, PIDConstants
from pathplannerlib.path import FlippingUtil
from lib import logger, utils
from lib.classes import (
  RobotType,
  Alliance, 
  PID,
  Zone,
  MotorModel,
  SwerveModuleGearKit,
  SwerveModuleConstants, 
  SwerveModuleConfig, 
  SwerveModuleLocation, 
  PoseAlignmentConstants,
  HeadingAlignmentConstants,
  PoseSensorConfig
)
from core.classes import Target
import lib.constants

_aprilTagFieldLayout = AprilTagFieldLayout(f'{ wpilib.getDeployDirectory() }/localization/2026-rebuilt-andymark.json')

class Subsystems:
  class Drive:
    BUMPER_LENGTH: units.meters = units.inchesToMeters(19.5)
    BUMPER_WIDTH: units.meters = units.inchesToMeters(19.5)
    WHEEL_BASE: units.meters = units.inchesToMeters(9.125)
    TRACK_WIDTH: units.meters = units.inchesToMeters(9.125)
    
    _drivingMotorModel = MotorModel.NEOVortex
    _swerveModuleGearKit = SwerveModuleGearKit.High
    
    _swerveModuleConstants = SwerveModuleConstants(
      wheelDiameter = units.inchesToMeters(3.0),
      drivingMotorControllerType = SparkLowLevel.SparkModel.kSparkMax,
      drivingMotorType = SparkLowLevel.MotorType.kBrushless,
      drivingMotorFreeSpeed = lib.constants.Motors.MOTOR_FREE_SPEEDS[_drivingMotorModel],
      drivingMotorReduction = lib.constants.Drive.SWERVE_MODULE_GEAR_RATIOS[_swerveModuleGearKit],
      drivingMotorCurrentLimit = 80,
      drivingMotorPID = PID(0.04, 0, 0),
      turningMotorCurrentLimit = 20,
      turningMotorPID = PID(1.0, 0, 0),
      turningMotorAbsoluteEncoderConfig = AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoder()
    )

    SWERVE_MODULE_CONFIGS: tuple[SwerveModuleConfig, SwerveModuleConfig, SwerveModuleConfig, SwerveModuleConfig] = (
      SwerveModuleConfig(SwerveModuleLocation.FrontLeft, 2, 3, -90, Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), _swerveModuleConstants),
      SwerveModuleConfig(SwerveModuleLocation.FrontRight, 4, 5, 0, Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), _swerveModuleConstants),
      SwerveModuleConfig(SwerveModuleLocation.RearLeft, 6, 7, 180, Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), _swerveModuleConstants),
      SwerveModuleConfig(SwerveModuleLocation.RearRight, 8, 9, 90, Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2), _swerveModuleConstants)
    )

    DRIVE_KINEMATICS = SwerveDrive4Kinematics(*(c.translation for c in SWERVE_MODULE_CONFIGS))

    TRANSLATION_MAX_VELOCITY: units.meters_per_second = lib.constants.Drive.SWERVE_MODULE_FREE_SPEEDS[_drivingMotorModel][_swerveModuleGearKit] * 0.5
    ROTATION_MAX_VELOCITY: units.degrees_per_second = 540.0

    TARGET_POSE_ALIGNMENT_CONSTANTS = PoseAlignmentConstants(
      translationPID = PID(3.0, 0, 0),
      translationMaxVelocity = 2.0,
      translationPositionTolerance = 0.025,
      rotationPID = PID(3.0, 0, 0),
      rotationMaxVelocity = 720.0,
      rotationPositionTolerance = 0.5
    )

    TARGET_HEADING_ALIGNMENT_CONSTANTS = HeadingAlignmentConstants(
      rotationPID = PID(0.01, 0, 0), 
      rotationPositionTolerance = 0.5
    )

    DRIFT_CORRECTION_CONSTANTS = HeadingAlignmentConstants(
      rotationPID = PID(0.01, 0, 0), 
      rotationPositionTolerance = 0.5
    )

    PATHPLANNER_ROBOT_CONFIG = RobotConfig.fromGUISettings()
    PATHPLANNER_CONTROLLER = PPHolonomicDriveController(PIDConstants(5.0, 0, 0), PIDConstants(5.0, 0, 0))

    INPUT_LIMIT_DEMO: units.percent = 0.5
    INPUT_RATE_LIMIT_DEMO: units.percent = 0.5

class Services:
  class Localization:
    MAX_TARGET_AMBIGUITY: units.percent = 0.2
    MAX_TARGET_REPROJECTION_ERROR: float = 1.0
    MAX_TARGET_DISTANCE: units.meters = 5.0
    MAX_POSE_CHANGE: units.meters = 1.0
    STDDEV_XY_COEFF: float = 0.08
    STDDEV_Z_COEFF: float = 0.1
    STDDEV_TARGET_AMBIGUITY_SCALE_FACTOR: float = 5.0
    STDDEV_TARGET_REPROJECTION_ERROR_SCALE_FACTOR: float = 2.5
    VALID_POSE_SENSOR_RESULT_TIMEOUT: units.seconds = 0.3

  class Targeting:
    pass

class Sensors: 
  class Gyro:
    class NAVX2:
      COM_TYPE = AHRS.NavXComType.kMXP_SPI
  
  class Pose:
    POSE_SENSOR_CONFIGS: tuple[PoseSensorConfig, ...] = (
      # PoseSensorConfig(
      #   name = "Front",
      #   transform = Transform3d(
      #     Translation3d(x = units.inchesToMeters(4.25), y = units.inchesToMeters(-1.77), z = units.inchesToMeters(9.47)), 
      #     Rotation3d(roll = units.degreesToRadians(-0.18), pitch = units.degreesToRadians(-32.77), yaw = units.degreesToRadians(-0.18))
      #   ),
      #   stream = "http://10.28.81.6:1182/?action=stream", 
      #   aprilTagFieldLayout = _aprilTagFieldLayout
      # ),
    )

class Cameras:
  DRIVER_STREAM = "http://10.28.81.6:1182/?action=stream"

class Controllers:
  DRIVER_CONTROLLER_PORT: int = 0
  OPERATOR_CONTROLLER_PORT: int = 1
  INPUT_DEADBAND: units.percent = 0.1

class Game:
  class Robot:
    TYPE = RobotType.Practice
    NAME: str = "MiniBot (Black)"

  class Commands:
    pass

  class Field:
    LENGTH = _aprilTagFieldLayout.getFieldLength()
    WIDTH = _aprilTagFieldLayout.getFieldWidth()
    ZONE = Zone(start = Translation2d(0, 0), end = Translation2d(LENGTH, WIDTH))

    class Targets:
      TARGETS: dict[Alliance, dict[Target, Pose3d]] = {
        Alliance.Blue: {
          Target.Hub: Pose3d(4.625, 4.030, 1.263, Rotation3d(Rotation2d.fromDegrees(0)))
        },
        Alliance.Red: {}
      }

      for target in TARGETS[Alliance.Blue]:
        pose = FlippingUtil.flipFieldPose(TARGETS[Alliance.Blue][target].toPose2d())
        TARGETS[Alliance.Red][target] = Pose3d(pose.X(), pose.Y(), TARGETS[Alliance.Blue][target].Z(), Rotation3d(pose.rotation()))

      TARGET_ZONES: dict[Alliance, dict[Target, Zone]] = {
        Alliance.Blue: {
          Target.Hub: Zone(start = Translation2d(0.0, 0.0), end = Translation2d(4.4, 8.0))
        },
        Alliance.Red: {}
      }

      for target in TARGET_ZONES[Alliance.Blue]:
        zone = TARGET_ZONES[Alliance.Blue][target]
        TARGET_ZONES[Alliance.Red][target] = Zone(
          FlippingUtil.flipFieldPose(Pose2d(zone.end.X(), zone.end.Y(), Rotation2d())).translation(), 
          FlippingUtil.flipFieldPose(Pose2d(zone.start.X(), zone.start.Y(), Rotation2d())).translation()
        )