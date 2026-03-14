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
    kMaxSpeedMps = 1.5          # capped change to 6 when ready.
    kMaxAngularSpeedRps = 2 * math.pi

    kMagnitudeSlewRate  = 3.0
    kRotationalSlewRate = 2.0
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

    kFrontLeftDriveId  = 11
    kFrontLeftTurnId   = 10
    kFrontRightDriveId = 13
    kFrontRightTurnId  = 12
    kRearLeftDriveId   = 17
    kRearLeftTurnId    = 18
    kRearRightDriveId  = 7
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

    kDriveCurrentLimit = 50  # Amps
    kTurnCurrentLimit  = 20  # Amps

    # FIXED: was 241, 158 (pixel coords — way off field).
    # Set to a real field position in meters, or (0, 0) as a safe default.
    # Update these to your actual starting pose before each match.
    k_start_x = 2.0   # meters from blue alliance wall
    k_start_y = 4.0   # meters from bottom wall (mid-field height)


# ─────────────────────────────────────────────────────────────────────────────
# SHOOTER
# ─────────────────────────────────────────────────────────────────────────────
class ShooterConstants:
    kShooterMotor1Id = 1
    kShooterMotor2Id = 3
    kFeederMotorId   = 16

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
    kLeftMotorId  = 15
    kRightMotorId = 14
    #kFuelSensorChannel = 10

    kIntakeSpeed = 0.8
    kReverseSpeed = -0.5
    kCurrentLimit = 30


# ─────────────────────────────────────────────────────────────────────────────
# CLIMBER
# ─────────────────────────────────────────────────────────────────────────────
class ClimberConstants:
    kClimberMotorId = 4
    kClimberMotor2Id = 5
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
    kPxController = 5.0
    kPyController = 5.0

    kPThetaController = 5.0

    kMaxSpeedMps   = 1.5 #change to 3 when ready
    kMaxAccelMps2  = 2.0

    kRobotMassKg = 55.0
    kRobotMOI    = 4.5

    kWheelCOF = 1.0