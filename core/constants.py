import wpilib
from wpimath import units
from wpimath.geometry import Pose3d, Rotation3d, Translation2d, Rotation2d, Rectangle2d
from wpimath.kinematics import SwerveDrive4Kinematics
from robotpy_apriltag import AprilTagFieldLayout
import navx
from rev import SparkLowLevel, AbsoluteEncoderConfig
from pathplannerlib.config import RobotConfig
from pathplannerlib.controller import PPHolonomicDriveController, PIDConstants
from pathplannerlib.path import FlippingUtil
from lib import logger, telemetry, utils
from lib.classes import (
  RobotType,
  Alliance, 
  PID,
  State,
  SpeedMode,
  DriveOrientation,
  MotorModel,
  SwerveDriveModuleGearKit,
  SwerveDriveModuleConfigConstants, 
  SwerveDriveModuleConfig, 
  SwerveDriveModuleLocation, 
  PoseAlignmentConstants,
  HeadingAlignmentConstants,
  XboxControllerConfig,
  PoseSensorConfig
)
from core.classes import Target, Zone
import lib.constants

_aprilTagFieldLayout = AprilTagFieldLayout(f'{ wpilib.getDeployDirectory() }/localization/2026-rebuilt-andymark.json')

class Subsystems:
  class Drive:
    BUMPER_LENGTH: units.meters = units.inchesToMeters(19.5)
    BUMPER_WIDTH: units.meters = units.inchesToMeters(19.5)
    WHEEL_BASE: units.meters = units.inchesToMeters(9.125)
    TRACK_WIDTH: units.meters = units.inchesToMeters(9.125)
    
    _drivingMotorModel = MotorModel.NEOVortex
    _swerveDriveModuleGearKit = SwerveDriveModuleGearKit.High
    _swerveDriveModuleConstants = SwerveDriveModuleConfigConstants(
      drivingControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      drivingMotorType = SparkLowLevel.MotorType.kBrushless,
      drivingFreeSpeed = lib.constants.Motors.FREE_SPEEDS[_drivingMotorModel],
      drivingGearReduction = lib.constants.Drive.Swerve.GEAR_RATIOS[_swerveDriveModuleGearKit],
      drivingCurrentLimit = 60,
      drivingControlPID = PID(0.04, 0, 0),
      turningCurrentLimit = 20,
      turningControlPID = PID(1.0, 0, 0),
      turningEncoderConfig = AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoder(),
      wheelDiameter = units.inchesToMeters(3.0),
      telemetryName = "Robot/Subsystems/Drive/Modules"
    )

    SWERVE_DRIVE_MODULE_CONFIGS: tuple[SwerveDriveModuleConfig, SwerveDriveModuleConfig, SwerveDriveModuleConfig, SwerveDriveModuleConfig] = (
      SwerveDriveModuleConfig(SwerveDriveModuleLocation.FrontLeft, 2, 3, -90, Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), _swerveDriveModuleConstants),
      SwerveDriveModuleConfig(SwerveDriveModuleLocation.FrontRight, 4, 5, 0, Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), _swerveDriveModuleConstants),
      SwerveDriveModuleConfig(SwerveDriveModuleLocation.RearLeft, 6, 7, 180, Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), _swerveDriveModuleConstants),
      SwerveDriveModuleConfig(SwerveDriveModuleLocation.RearRight, 8, 9, 90, Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2), _swerveDriveModuleConstants)
    )
    SWERVE_DRIVE_KINEMATICS = SwerveDrive4Kinematics(*(c.chassisTranslation for c in SWERVE_DRIVE_MODULE_CONFIGS))

    TRANSLATION_MAX_VELOCITY: units.meters_per_second = lib.constants.Drive.Swerve.FREE_SPEEDS[_drivingMotorModel][_swerveDriveModuleGearKit] * 1.0
    ROTATION_MAX_VELOCITY: units.degrees_per_second = 720.0

    TARGET_POSE_ALIGNMENT_CONSTANTS = PoseAlignmentConstants(
      translationControlPID = PID(4.0, 0, 0),
      translationMaxVelocity = 3.2,
      translationPositionTolerance = 0.15,
      rotationControlPID = PID(4.0, 0, 0),
      rotationMaxVelocity = 720.0,
      rotationPositionTolerance = 5.0
    )

    TARGET_HEADING_ALIGNMENT_CONSTANTS = HeadingAlignmentConstants(
      rotationControlPID = PID(0.01, 0, 0), 
      rotationPositionTolerance = 1.0
    )

    DRIFT_CORRECTION_CONSTANTS = HeadingAlignmentConstants(
      rotationControlPID = PID(0.01, 0, 0), 
      rotationPositionTolerance = 0.5
    )

    PATHPLANNER_ROBOT_CONFIG = RobotConfig.fromGUISettings()
    PATHPLANNER_CONTROLLER = PPHolonomicDriveController(PIDConstants(5.0, 0, 0), PIDConstants(5.0, 0, 0))

    INPUT_LIMIT_DEMO: units.percent = 0.5
    INPUT_RATE_LIMIT_DEMO: units.percent = 0.5

    SPEED_MODE = SpeedMode.Competition
    DRIVE_ORIENTATION = DriveOrientation.Field
    DRIFT_CORRECTION = State.Enabled

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
    NAVX_PORT = navx.AHRS.NavXComType.kMXP_SPI
  
  class Pose:
    POSE_SENSOR_CONFIGS: tuple[PoseSensorConfig, ...] = (
      # PoseSensorConfig(
      #   cameraName = "Front", 
      #   transform = Transform3d(
      #     Translation3d(x = units.inchesToMeters(-0.5), y = units.inchesToMeters(14.5), z = units.inchesToMeters(18.0)),
      #     Rotation3d(roll = units.degreesToRadians(0), pitch = units.degreesToRadians(-5.5), yaw = units.degreesToRadians(88.0))
      #   ),
      #   stream = "http://10.28.81.6:1186/?action=stream",
      #   aprilTagFieldLayout = _aprilTagFieldLayout,
      #   telemetryName = "Robot/Sensors/Pose"
      # ),
    )

class Cameras:
  DRIVER_STREAM = "http://10.28.81.6:1182/?action=stream"

class Controllers:
  DRIVER_CONTROLLER_CONFIG = XboxControllerConfig(port = 0, inputDeadband = 0.1, telemetryName = "Robot/Controllers/Driver")
  # OPERATOR_CONTROLLER_CONFIG = XboxControllerConfig(port = 1, inputDeadband = 0.1, telemetryName = "Robot/Controllers/Operator")
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
    BOUNDS = Rectangle2d(Translation2d(0, 0), Translation2d(LENGTH, WIDTH))

    TARGETS: dict[Alliance, dict[Target, Pose3d]] = {
      Alliance.Blue: {
        Target.Default: Pose3d(4.625, 4.030, 1.263, Rotation3d(Rotation2d.fromDegrees(0)))
      },
      Alliance.Red: {}
    }
    for target in TARGETS[Alliance.Blue]:
      pose = FlippingUtil.flipFieldPose(TARGETS[Alliance.Blue][target].toPose2d())
      TARGETS[Alliance.Red][target] = Pose3d(pose.X(), pose.Y(), TARGETS[Alliance.Blue][target].Z(), Rotation3d(pose.rotation()))

    ZONES: dict[Alliance, dict[Zone, Rectangle2d]] = {
      Alliance.Blue: {
        Zone.Default: Rectangle2d(Translation2d(0, 0), Translation2d(4.400, 4.022))
      },
      Alliance.Red: {}
    }
    for zone in ZONES[Alliance.Blue]:
      rectangle = ZONES[Alliance.Blue][zone]
      ZONES[Alliance.Red][zone] = Rectangle2d(FlippingUtil.flipFieldPose(rectangle.center()), rectangle.xwidth, rectangle.ywidth)