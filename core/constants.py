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
  TargetAlignmentConstants,
  RotationAlignmentConstants,
  PoseSensorConfig
)
from core.classes import Target
import lib.constants

_aprilTagFieldLayout = AprilTagFieldLayout(f'{ wpilib.getDeployDirectory() }/localization/2026-rebuilt-andymark.json')
_pathPlannerRobotConfig = RobotConfig.fromGUISettings()

class Subsystems:
  class Drive:
    BUMPER_LENGTH: units.meters = units.inchesToMeters(19.5)
    BUMPER_WIDTH: units.meters = units.inchesToMeters(19.5)
    WHEEL_BASE: units.meters = units.inchesToMeters(9.125)
    TRACK_WIDTH: units.meters = units.inchesToMeters(9.125)
    
    TRANSLATION_SPEED_MAX: units.meters_per_second = 4.46
    ROTATION_SPEED_MAX: units.degrees_per_second = 720.0

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

    PATHPLANNER_ROBOT_CONFIG = _pathPlannerRobotConfig
    PATHPLANNER_CONTROLLER = PPHolonomicDriveController(PIDConstants(5.0, 0, 0), PIDConstants(5.0, 0, 0))

    TARGET_ALIGNMENT_CONSTANTS = TargetAlignmentConstants(
      translationPID = PID(5.0, 0, 0),
      translationMaxVelocity = 1.6,
      translationMaxAcceleration = 0.8,
      translationPositionTolerance = 0.025,
      translationVelocityTolerance = 0.1,
      rotationPID = PID(5.0, 0, 0),
      rotationMaxVelocity = 720.0,
      rotationMaxAcceleration = 540.0,
      rotationPositionTolerance = 0.25,
      rotationVelocityTolerance = 45.0
    )

    TARGET_LOCK_CONSTANTS = RotationAlignmentConstants(
      rotationPID = PID(0.01, 0, 0), 
      rotationPositionTolerance = 0.25,
      rotationAlignmentOffset = 0
    )

    DRIFT_CORRECTION_CONSTANTS = RotationAlignmentConstants(
      rotationPID = PID(0.01, 0, 0), 
      rotationPositionTolerance = 0.25
    )

    INPUT_LIMIT_DEMO: units.percent = 0.5
    INPUT_RATE_LIMIT_DEMO: units.percent = 0.5

class Services:
  class Localization:
    VISION_MAX_POSE_AMBIGUITY: units.percent = 0.2
    VISION_ESTIMATE_STANDARD_DEVIATIONS: tuple[units.meters, units.meters, units.radians] = (0.3, 0.3, units.degreesToRadians(15.0))

class Sensors: 
  class Gyro:
    class NAVX2:
      COM_TYPE = AHRS.NavXComType.kUSB1
  
  class Pose:
    POSE_SENSOR_CONFIGS: tuple[PoseSensorConfig, ...] = (
      PoseSensorConfig(
        name = "Front",
        transform = Transform3d(Translation3d(0.105985, -0.057490, 0.339597), Rotation3d(-0.005636, -0.158088, 0.020506)),
        stream = "http://10.28.81.6:1182/?action=stream", 
        aprilTagFieldLayout = _aprilTagFieldLayout
      ),
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
    AUTO_TARGET_ALIGNMENT_TIMEOUT: units.seconds = 1.5

  class Field:
    LENGTH = _aprilTagFieldLayout.getFieldLength()
    WIDTH = _aprilTagFieldLayout.getFieldWidth()
    BOUNDS = (Translation2d(0, 0), Translation2d(LENGTH, WIDTH))

    class Targets:
      TARGETS: dict[Alliance, dict[Target, Pose3d]] = {
        Alliance.Red: {
          Target.Hub: Pose3d(11.918, 4.032, 1.263, Rotation3d(Rotation2d.fromDegrees(0))).transformBy(
            Transform3d(units.inchesToMeters(0), units.inchesToMeters(0), units.inchesToMeters(0), Rotation3d(Rotation2d.fromDegrees(0)))
          ),
        },
        Alliance.Blue: {
          Target.Hub: Pose3d(4.623, 4.032, 1.263, Rotation3d(Rotation2d.fromDegrees(0))).transformBy(
            Transform3d(units.inchesToMeters(0), units.inchesToMeters(0), units.inchesToMeters(0), Rotation3d(Rotation2d.fromDegrees(0)))
          ),
        }
      }
