"""
Team 1504 Desperate Penguins - Constants
2026 Season | FRC Game: Rebuilt

All hardware IDs, tuning values, and physical measurements in one place.
Group by subsystem so it's easy to find things.
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
    kDriveDeadband = 0.08
    kRotDeadband = 0.12          # Slightly tighter on rotation to prevent drift
    kSlowModeMultiplier = 0.30   # 30% speed while slow mode held


# ─────────────────────────────────────────────────────────────────────────────
# SWERVE DRIVE
# ─────────────────────────────────────────────────────────────────────────────
class DriveConstants:
    # Physical limits (what the robot is ALLOWED to do in teleop)
    kMaxSpeedMps = 4.8          # meters per second
    kMaxAngularSpeedRps = 2 * math.pi   # radians per second

    # Slew rate limits — per axis (X and Y slewed independently)
    # Higher = more responsive, lower = smoother.
    # 4.0 is a good starting point; raise toward 6.0 if still sluggish.
    kMagnitudeSlewRate = 4.0    # units/second per axis
    kRotationalSlewRate = 3.0   # units/second

    # Slow mode — fraction of max speed applied when slow mode button is held
    kSlowModeMultiplier = 0.3   # 30% of full speed

    # Chassis geometry (center-to-center of wheels, in meters)
    kTrackWidth = 0.5715
    kWheelBase  = 0.5715

    # Angular offsets of each module relative to chassis (radians)
    kFrontLeftChassisOffset  = -math.pi / 2
    kFrontRightChassisOffset = 0.0
    kRearLeftChassisOffset   = math.pi
    kRearRightChassisOffset  = math.pi / 2

    # Absolute encoder offsets (tuned per robot - check with REV Hardware Client)
    kFrontLeftEncoderOffset  = 0.8546
    kFrontRightEncoderOffset = 0.665
    kRearLeftEncoderOffset   = 0.803
    kRearRightEncoderOffset  = 0.1814

    # ── CAN IDs ───────────────────────────────────────────────────
    kFrontLeftDriveId  = 5
    kFrontLeftTurnId   = 6
    kFrontRightDriveId = 7
    kFrontRightTurnId  = 8
    kRearLeftDriveId   = 3
    kRearLeftTurnId    = 4
    kRearRightDriveId  = 1
    kRearRightTurnId   = 2

    # ── Gearing / conversion ──────────────────────────────────────
    kDrivePinionTeeth = 14   # 12T / 13T / 14T options on MAXSwerve
    kDriveMotorFreeSpeedRps = 5676.0 / 60  # NEO free speed
    kWheelDiameterMeters = 0.0762
    kWheelCircumferenceMeters = kWheelDiameterMeters * math.pi
    kDriveMotorReduction = (45.0 * 22) / (kDrivePinionTeeth * 15)
    kDriveWheelFreeSpeedRps = (kDriveMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDriveMotorReduction

    kDriveEncoderPositionFactor = kWheelCircumferenceMeters / kDriveMotorReduction  # meters
    kDriveEncoderVelocityFactor = kDriveEncoderPositionFactor / 60.0                # m/s

    kTurnEncoderPositionFactor = 2 * math.pi   # radians
    kTurnEncoderVelocityFactor = (2 * math.pi) / 60.0
    kTurnEncoderInverted = True

    # ── PID (SparkMax onboard) ─────────────────────────────────────
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

    # Brake mode on both drive and turn:
    # - Drive brake: robot decelerates quickly and holds position when input stops
    # - Turn brake: wheels hold their angle instead of drifting
    kDriveIdleMode = SparkMaxConfig.IdleMode.kBrake
    kTurnIdleMode  = SparkMaxConfig.IdleMode.kBrake

    kDriveCurrentLimit = 50   # Amps
    kTurnCurrentLimit  = 20   # Amps


# ─────────────────────────────────────────────────────────────────────────────
# SHOOTER  (Phoenix6 TalonFX — Kraken X60 or Falcon 500)
# ─────────────────────────────────────────────────────────────────────────────
class ShooterConstants:
    kShooterMotor1Id = 20   # TalonFX CAN ID
    kShooterMotor2Id = 21   # TalonFX CAN ID (follows / opposite polarity)
    kFeederMotorId   = 10   # SparkMax CAN ID (NEO 550 or similar)

    # Velocity PID (slot 0 on TalonFX — tuned in Phoenix Tuner X)
    kShooterP  = 0.5
    kShooterI  = 0.0
    kShooterD  = 0.0
    kShooterKv = 0.12   # V per RPS - start here, tune with Tuner X

    # Tolerance for "at speed" check (RPS)
    kVelocityToleranceRps = 2.0

    # Feeder duty cycle
    kFeederSpeed = 0.6

    kFeederCurrentLimit = 20   # Amps — NEO 1.1
    kFlywheelCurrentLimit = 60  # Amps — NEO Vortex (SparkFlex)

    # Distance → shooter RPS lookup table
    # Format: (distance_meters, target_rps)
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
    kLeftMotorId  = 12   # SparkMax
    kRightMotorId = 11   # SparkMax
    kFuelSensorChannel = 9   # DIO

    kIntakeSpeed = 0.8
    kReverseSpeed = -0.5

    kCurrentLimit = 30  # Amps


# ─────────────────────────────────────────────────────────────────────────────
# CLIMBER
# ─────────────────────────────────────────────────────────────────────────────
class ClimberConstants:
    kClimberMotorId = 30   # TalonFX or SparkMax - update to match hardware
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
# AUTONOMOUS
# ─────────────────────────────────────────────────────────────────────────────
class AutoConstants:
    kMaxSpeedMps = 3.0
    kMaxAccelMps2 = 2.0
    kMaxAngularSpeedRps = math.pi
    kMaxAngularAccelRps2 = math.pi

    kPxController = 0.5
    kPyController = 0.5
    kPThetaController = 0.5


# ─────────────────────────────────────────────────────────────────────────────
# LEDs
# ─────────────────────────────────────────────────────────────────────────────
class LEDConstants:
    kLEDPort = 0      # PWM port on roboRIO
    kLEDLength = 60   # Number of LEDs in strip