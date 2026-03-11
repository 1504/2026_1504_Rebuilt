"""
Team 1504 Desperate Penguins - Constants
2026 Season | FRC Game: Rebuilt
"""

import math
import rev
import wpimath.trajectory
from rev import SparkMaxConfig


# ─────────────────────────────────────────────────────────────────────────────
# OPERATOR INTERFACE
# ─────────────────────────────────────────────────────────────────────────────
class OIConstants:
    kDriverPort = 0
    kOperatorPort = 1
    kDriveDeadband = 0.13
    kRotDeadband = 0.16
    kSlowModeMultiplier = 0.30


# ─────────────────────────────────────────────────────────────────────────────
# SWERVE DRIVE
# ─────────────────────────────────────────────────────────────────────────────
class DriveConstants:
    kMaxSpeedMps = 0.8 # was 4.8
    kMaxAngularSpeedRps = 2 * math.pi

    kMagnitudeSlewRate  = 3.0 # 4 works well, slowed down for now, can try 6
    kRotationalSlewRate = 2.0 # 3 works well, slowed down for now
    kSlowModeMultiplier = 0.3

    kTrackWidth = 0.5715
    kWheelBase  = 0.5715

    kFrontLeftChassisOffset  = -math.pi / 2
    kFrontRightChassisOffset = 0.0
    kRearLeftChassisOffset   = math.pi
    kRearRightChassisOffset  = math.pi / 2

    kFrontLeftEncoderOffset  = 0.8546
    kFrontRightEncoderOffset = 0.665
    kRearLeftEncoderOffset   = 0.803
    kRearRightEncoderOffset  = 0.1814

    kFrontLeftDriveId  = 5
    kFrontLeftTurnId   = 6
    kFrontRightDriveId = 7
    kFrontRightTurnId  = 8
    kRearLeftDriveId   = 3
    kRearLeftTurnId    = 4
    kRearRightDriveId  = 1
    kRearRightTurnId   = 2

    kDrivePinionTeeth = 14
    kDriveMotorFreeSpeedRps = 5676.0 / 60
    kWheelDiameterMeters = 0.0762
    kWheelCircumferenceMeters = kWheelDiameterMeters * math.pi
    kDriveMotorReduction = (45.0 * 22) / (kDrivePinionTeeth * 15)
    kDriveWheelFreeSpeedRps = (kDriveMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDriveMotorReduction

    kDriveEncoderPositionFactor = kWheelCircumferenceMeters / kDriveMotorReduction
    kDriveEncoderVelocityFactor = kDriveEncoderPositionFactor / 60.0

    kTurnEncoderPositionFactor = 2 * math.pi
    kTurnEncoderVelocityFactor = (2 * math.pi) / 60.0
    kTurnEncoderInverted = True

    kDriveP  = 0.04
    kDriveI  = 0.0
    kDriveD  = 0.0
    kDriveFF = 1.0 / kDriveWheelFreeSpeedRps

    kTurnP  = 2.0
    kTurnI  = 0.0
    kTurnD  = 0.0
    kTurnFF = 0.0

    kDriveMinOutput = -1.0
    kDriveMaxOutput =  1.0
    kTurnMinOutput  = -1.0
    kTurnMaxOutput  =  1.0

    kDriveIdleMode = SparkMaxConfig.IdleMode.kBrake
    kTurnIdleMode  = SparkMaxConfig.IdleMode.kBrake

    kDriveCurrentLimit = 50
    kTurnCurrentLimit  = 20


# ─────────────────────────────────────────────────────────────────────────────
# SHOOTER
# ─────────────────────────────────────────────────────────────────────────────
class ShooterConstants:
    kShooterMotor1Id = 20
    kShooterMotor2Id = 21
    kFeederMotorId   = 10

    kShooterP  = 0.5
    kShooterI  = 0.0
    kShooterD  = 0.0
    kShooterKv = 0.12

    kVelocityToleranceRps = 2.0
    kFeederSpeed = 0.6
    kFeederCurrentLimit = 20
    kFlywheelCurrentLimit = 60

    kShooterTable = [
        (1.5,  35.0),
        (2.0,  40.0),
        (2.5,  45.0),
        (3.0,  50.0),
        (3.5,  55.0),
    ]


# ─────────────────────────────────────────────────────────────────────────────
# INTAKE
# ─────────────────────────────────────────────────────────────────────────────
class IntakeConstants:
    kLeftMotorId  = 12
    kRightMotorId = 11
    kFuelSensorChannel = 9

    kIntakeSpeed = 0.8
    kReverseSpeed = -0.5
    kCurrentLimit = 30


# ─────────────────────────────────────────────────────────────────────────────
# CLIMBER
# ─────────────────────────────────────────────────────────────────────────────
class ClimberConstants:
    kClimberMotorId = 30
    kClimbSpeed = 0.8
    kDescendSpeed = -0.5
    kCurrentLimit = 40


# ─────────────────────────────────────────────────────────────────────────────
# VISION
# ─────────────────────────────────────────────────────────────────────────────
class VisionConstants:
    kLimelightName = "limelight"
    kCameraToRobotX = 0.0
    kCameraToRobotY = 0.0
    kCameraToRobotZ = 0.5
    kCameraRoll  = 0.0
    kCameraPitch = 25.0
    kCameraYaw   = 0.0

    kSingleTagStdDevs = (4.0, 4.0, 8.0)
    kMultiTagStdDevs  = (0.5, 0.5, 1.0)


# ─────────────────────────────────────────────────────────────────────────────
# AUTONOMOUS  (PathPlanner)
# ─────────────────────────────────────────────────────────────────────────────
class AutoConstants:
    # Translation PID — how aggressively PathPlanner corrects X/Y error.
    # Start at 5.0, increase if the robot lags behind the path.
    kPxController = 5.0
    kPyController = 5.0   # must equal kPxController for holonomic

    # Rotation PID — how aggressively PathPlanner corrects heading error.
    # Start at 5.0, increase if the robot's heading drifts during paths.
    kPThetaController = 5.0

    # PathPlanner path speed limits (override per-path in the GUI if needed)
    kMaxSpeedMps   = 3.0
    kMaxAccelMps2  = 2.0

    # Robot physical properties — used for PathPlanner dynamics model.
    # kRobotMassKg: weigh your robot (frame + battery + mechanisms).
    # kRobotMOI:    moment of inertia — estimate = 0.5 * mass * (half_diagonal)^2
    #               half_diagonal for 0.5715 m square chassis ≈ 0.404 m
    #               So MOI ≈ 0.5 * 55 * 0.404^2 ≈ 4.5  (tune from there)
    kRobotMassKg = 55.0   # kg — update after weighing
    kRobotMOI    = 4.5    # kg·m² — update after measuring or using SysId

    # Coefficient of friction between wheel and carpet.
    # 1.0 is a common starting value for colsons/neoprene on FRC carpet.
    kWheelCOF = 1.0