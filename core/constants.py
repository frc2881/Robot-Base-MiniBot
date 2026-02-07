import wpilib
from wpimath import units
from wpimath.geometry import Pose3d, Transform3d, Translation3d, Rotation3d, Translation2d, Rotation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from robotpy_apriltag import AprilTagFieldLayout
from navx import AHRS
from rev import SparkLowLevel, AbsoluteEncoderConfig
from pathplannerlib.config import RobotConfig
from pathplannerlib.controller import PPHolonomicDriveController, PIDConstants
from lib import logger, utils
from lib.classes import (
  RobotType,
  Alliance, 
  PID, 
  MotorModel,
  SwerveModuleGearKit,
  SwerveModuleConstants, 
  SwerveModuleConfig, 
  SwerveModuleLocation, 
  PoseAlignmentConstants,
  HeadingAlignmentConstants,
  PoseSensorConfig,
  ObjectSensorConfig
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
    
    TRANSLATION_MAX_VELOCITY: units.meters_per_second = 4.46
    ROTATION_MAX_VELOCITY: units.degrees_per_second = 720.0

    _swerveModuleConstants = SwerveModuleConstants(
      wheelDiameter = units.inchesToMeters(3.0),
      drivingMotorControllerType = SparkLowLevel.SparkModel.kSparkMax,
      drivingMotorType = SparkLowLevel.MotorType.kBrushless,
      drivingMotorFreeSpeed = lib.constants.Motors.MOTOR_FREE_SPEEDS[MotorModel.NEO],
      drivingMotorReduction = lib.constants.Drive.SWERVE_MODULE_GEAR_RATIOS[SwerveModuleGearKit.Medium],
      drivingMotorCurrentLimit = 80,
      drivingMotorPID = PID(0.04, 0, 0),
      turningMotorCurrentLimit = 20,
      turningMotorPID = PID(1.0, 0, 0),
      turningMotorAbsoluteEncoderConfig = AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoder
    )

    SWERVE_MODULE_CONFIGS: tuple[SwerveModuleConfig, ...] = (
      SwerveModuleConfig(SwerveModuleLocation.FrontLeft, 2, 3, -90, Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), _swerveModuleConstants),
      SwerveModuleConfig(SwerveModuleLocation.FrontRight, 4, 5, 0, Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), _swerveModuleConstants),
      SwerveModuleConfig(SwerveModuleLocation.RearLeft, 6, 7, 180, Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), _swerveModuleConstants),
      SwerveModuleConfig(SwerveModuleLocation.RearRight, 8, 9, 90, Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2), _swerveModuleConstants)
    )

    DRIVE_KINEMATICS = SwerveDrive4Kinematics(*(c.translation for c in SWERVE_MODULE_CONFIGS))

    PATHPLANNER_ROBOT_CONFIG = RobotConfig.fromGUISettings()
    PATHPLANNER_CONTROLLER = PPHolonomicDriveController(PIDConstants(5.0, 0, 0), PIDConstants(5.0, 0, 0))

    TARGET_POSE_ALIGNMENT_CONSTANTS = PoseAlignmentConstants(
      translationPID = PID(2.0, 0, 0),
      translationMaxVelocity = 1.5,
      translationMaxAcceleration = 0.75,
      translationPositionTolerance = 0.025,
      rotationPID = PID(3.0, 0, 0),
      rotationMaxVelocity = 720.0,
      rotationMaxAcceleration = 540.0,
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

    INPUT_LIMIT_DEMO: units.percent = 0.5
    INPUT_RATE_LIMIT_DEMO: units.percent = 0.5

class Services:
  class Localization:
    VISION_MAX_POSE_AMBIGUITY: units.percent = 0.2
    VISION_MAX_ESTIMATED_POSE_DELTA: units.meters = 3.0
    VISION_ESTIMATE_MULTI_TAG_STANDARD_DEVIATIONS: tuple[units.meters, units.meters, units.radians] = (0.05, 0.05, units.degreesToRadians(2.5))
    VISION_ESTIMATE_SINGLE_TAG_STANDARD_DEVIATIONS: tuple[units.meters, units.meters, units.radians] = (0.3, 0.3, units.degreesToRadians(15.0))

class Sensors: 
  class Gyro:
    class NAVX2:
      COM_TYPE = AHRS.NavXComType.kUSB1
  
  class Pose:
    POSE_SENSOR_CONFIGS: tuple[PoseSensorConfig, ...] = (
      PoseSensorConfig(
        name = "Front",
        transform = Transform3d(Translation3d(0.107311, -0.050843, 0.264506), Rotation3d(0.001834, -0.569486, -0.027619)),
        stream = "http://10.28.81.6:1182/?action=stream", 
        aprilTagFieldLayout = _aprilTagFieldLayout
      ),
    )

  class Object:
    OBJECT_SENSOR_CONFIG = ObjectSensorConfig(
      name = "Fuel", 
      transform = Transform3d(Translation3d(units.inchesToMeters(0), units.inchesToMeters(-7.0), units.inchesToMeters(22.0)), Rotation3d(0, units.degreesToRadians(6.6), units.degreesToRadians(15.0))),
      stream = "http://10.28.81.6:1182/?action=stream",
      objectHeight = units.inchesToMeters(5.71)
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
    NAME: str = "MiniBot"

  class Commands:
    AUTO_ALIGNMENT_TIMEOUT: units.seconds = 1.5

  class Field:
    LENGTH = _aprilTagFieldLayout.getFieldLength()
    WIDTH = _aprilTagFieldLayout.getFieldWidth()
    BOUNDS = (Translation2d(0, 0), Translation2d(LENGTH, WIDTH))

    class Targets:
      TARGETS: dict[Alliance, dict[Target, Pose3d]] = {
        Alliance.Red: {
          Target.Hub: Pose3d(11.918, 4.032, 1.263, Rotation3d(Rotation2d.fromDegrees(180))),
          Target.CornerLeft: Pose3d(14.000, 4.032, 0, Rotation3d(Rotation2d.fromDegrees(90))),
          Target.CornerRight: Pose3d(14.000, 4.032, 0, Rotation3d(Rotation2d.fromDegrees(-90))),
          Target.TowerLeft: Pose3d(14.000, 4.032, 0, Rotation3d(Rotation2d.fromDegrees(180))),
          Target.TowerRight: Pose3d(14.000, 4.032, 0, Rotation3d(Rotation2d.fromDegrees(180))),
          Target.TrenchLeft: Pose3d(14.000, 4.032, 0, Rotation3d(Rotation2d.fromDegrees(90))),
          Target.TrenchRight: Pose3d(14.000, 4.032, 0, Rotation3d(Rotation2d.fromDegrees(-90))),
          Target.Outpost: Pose3d(14.000, 4.032, 0, Rotation3d(Rotation2d.fromDegrees(180))),
          Target.Depot: Pose3d(14.000, 4.032, 0, Rotation3d(Rotation2d.fromDegrees(-90)))
        },
        Alliance.Blue: {
          Target.Hub: Pose3d(4.623, 4.032, 1.263, Rotation3d(Rotation2d.fromDegrees(0))),
          Target.CornerLeft: Pose3d(0.280, 7.790, 0, Rotation3d(Rotation2d.fromDegrees(-45))),
          Target.CornerRight: Pose3d(0.280, 0.280, 0, Rotation3d(Rotation2d.fromDegrees(45))),
          Target.TowerLeft: Pose3d(1.370, 4.180, 0, Rotation3d(Rotation2d.fromDegrees(180))),
          Target.TowerRight: Pose3d(1.370, 3.320, 0, Rotation3d(Rotation2d.fromDegrees(180))),
          Target.TrenchLeft: Pose3d(3.664, 6.535, 0, Rotation3d(Rotation2d.fromDegrees(-90))),
          Target.TrenchRight: Pose3d(3.664, 1.600, 0, Rotation3d(Rotation2d.fromDegrees(90))),
          Target.Outpost: Pose3d(0.280, 0.650, 0, Rotation3d(Rotation2d.fromDegrees(0))),
          Target.Depot: Pose3d(0.350, 5.125, 0, Rotation3d(Rotation2d.fromDegrees(0)))
        }
      }
