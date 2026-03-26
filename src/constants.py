"""
Team 1504 Desperate Penguins - Constants
2026 Season | FRC Game: Rebuilt
"""

import math
from wpimath.units import inchesToMeters
import wpimath.units
import wpimath.trajectory
from rev import ClosedLoopSlot, SparkMaxConfig


# ─────────────────────────────────────────────────────────────────────────────
# OPERATOR INTERFACE
# ─────────────────────────────────────────────────────────────────────────────
class OIConstants:
    kDriverPort          = 0
    kOperatorPort        = 1
    kDriveDeadband       = 0.13
    kRotDeadband         = 0.16
    kSlowModeMultiplier  = 0.30


# ─────────────────────────────────────────────────────────────────────────────
# SWERVE DRIVE
# ─────────────────────────────────────────────────────────────────────────────
class DriveConstants:
    # Teleop speed cap — conservative while tuning. Raise toward kMaxAutoSpeedMps
    # once the robot feels stable.
    kMaxSpeedMps         = 4.8
    # True hardware ceiling — used for PathPlanner desaturation and fallback config.
    # Never use kMaxSpeedMps for auto; PP feedforward will be wrong.
    kMaxAutoSpeedMps     = 4.8
    kMaxAngularSpeedRps  = 2 * math.pi

    kMagnitudeSlewRate   = 3.0
    kRotationalSlewRate  = 2.0
    kSlowModeMultiplier  = 0.3

    kTrackWidth = 0.6155
    kWheelBase  = 0.6155

    kFrontLeftChassisOffset  = -math.pi / 2
    kFrontRightChassisOffset = 0.0
    kRearLeftChassisOffset   = math.pi
    kRearRightChassisOffset  = math.pi / 2

    # ⚠️ VERIFY THESE with REV Hardware Client after any motor swap.
    # Wrong offsets = modules fighting each other or driving at wrong angles.
    # To check: put robot on blocks, zero each module in REV client, update below.
    kFrontLeftEncoderOffset  = 0.988
    kFrontRightEncoderOffset = 0.552
    kRearLeftEncoderOffset   = 0.380
    kRearRightEncoderOffset  = 0.480

    kFrontLeftDriveId  = 11
    kFrontLeftTurnId   = 10
    kFrontRightDriveId = 13
    kFrontRightTurnId  = 12
    kRearLeftDriveId   = 17
    kRearLeftTurnId    = 18
    kRearRightDriveId  = 7
    kRearRightTurnId   = 2

    kDrivePinionTeeth       = 14
    kDriveMotorFreeSpeedRps = 5676.0 / 60
    kWheelDiameterMeters    = 0.0762
    kWheelCircumferenceMeters = kWheelDiameterMeters * math.pi
    kDriveMotorReduction    = (45.0 * 22) / (kDrivePinionTeeth * 15)
    kDriveWheelFreeSpeedRps = (
        kDriveMotorFreeSpeedRps * kWheelCircumferenceMeters / kDriveMotorReduction
    )

    kDriveEncoderPositionFactor = kWheelCircumferenceMeters / kDriveMotorReduction
    kDriveEncoderVelocityFactor = kDriveEncoderPositionFactor / 60.0

    kTurnEncoderPositionFactor = 2 * math.pi
    kTurnEncoderVelocityFactor = (2 * math.pi) / 60.0
    kTurnEncoderInverted       = True

    kDriveP  = 0.5
    kDriveI  = 0.0
    kDriveD  = 0.0
    kDriveFF = 1.0 / kDriveWheelFreeSpeedRps
    #kDriveFF= 1.5

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

    # The RoboRIO faces LEFT relative to the robot's forward direction.
    # NavX plugged into MXP reports yaw as if RoboRIO-forward = robot-forward,
    # so we add -90° to every yaw read to correct for the 90° mounting rotation.
    # ⚠️ If the robot drives SIDEWAYS on the field after deploying this code,
    #    flip the sign to +90.0 and redeploy.
    kGyroMountingOffsetDeg = 0

    # Starting pose. PathPlanner's resetOdom:true in .auto files will overwrite
    # this at auto start — it's only the fallback before the first path runs.
    k_start_x = 0
    k_start_y = 0


# ─────────────────────────────────────────────────────────────────────────────
# SHOOTER
# ─────────────────────────────────────────────────────────────────────────────
class ShooterConstants:
    kShooterMotor1Id = 1
    kShooterMotor2Id = 3
    kFeederMotorId   = 16
    kAgitatorMotorId = 6

    kShooterP  = 0.5
    kShooterI  = 0.0
    kShooterD  = 0.0
    kShooterKv = 0.12

    kVelocityToleranceRps       = 20.0
    kFeederSpeed                = -0.6
    kAgitatorSpeed              = -0.6
    kFeederCurrentLimit         = 60
    kFlywheelCurrentLimit       = 60
    kFlywheelStatorCurrentLimit = 80

    # ── Live velocity tuning via D-pad ────────────────────────────
    # D-pad Up/Down steps through RPS at practice; Left resets to default.
    # DriveToShootCommand uses ShootingConstants.kTargetRps as its fixed value,
    # but ShootCommand/SpinUpCommand use the shared _current_target_rps state
    # in shooter_commands.py which starts at kDefaultShooterRps.
    kDefaultShooterRps = 70.0   # what D-pad Left resets to
    kShooterRpsStep    = 2.0    # how much each D-pad press changes velocity
    kShooterMinRps     = 10.0   # floor
    kShooterMaxRps     = 80.0   # ceiling

    # Kept for _interpolate_rps() in shooter.py — not used by DriveToShootCommand.
    kShooterTable = [
        (1.5, 35.0),
        (2.0, 40.0),
        (2.5, 45.0),
        (3.0, 50.0),
        (3.5, 55.0),
    ]


# ─────────────────────────────────────────────────────────────────────────────
# INTAKE
# ─────────────────────────────────────────────────────────────────────────────
class IntakeConstants:
    kMotorId      = 9
    kMotorId2     = 15
    kIntakeSpeed  =  0.15
    kReverseSpeed = -0.15
    kCurrentLimit = 70


# ─────────────────────────────────────────────────────────────────────────────
# CLIMBER
# ─────────────────────────────────────────────────────────────────────────────
class ClimberConstants:
    kClimberMotorId  = 4
    kClimberMotor2Id = 5
    kClimbSpeed      =  0.8
    kDescendSpeed    = -0.5
    kCurrentLimit    = 40


# ─────────────────────────────────────────────────────────────────────────────
# VISION
# ─────────────────────────────────────────────────────────────────────────────
class VisionConstants:
    kLimelightName = "limelight"

    # Camera is flat (0° pitch), on the intake face, centered left-right.
    # Confirmed: lens center is 0.675 m from the floor.
    kCameraHeightMeters = 0.675
    kCameraPitch        = 0.0   # flat mount — do NOT use TY-based distance
    kCameraYaw          = 0.0   # centered, no horizontal offset

    # Hub tag center height: 44.25 in = 1.124 m per 2026 game manual.
    kTagHeightMeters = 1.124

    # Slightly tightened from field edges — readings at 0.0 are almost always noise.
    kFieldMinX = 0.5
    kFieldMaxX = 16.0
    kFieldMinY = 0.5
    kFieldMaxY = 7.7

    # Trust XY at close range. Rotation is always 9999 — MegaTag2 fuses the
    # gyro for heading, so we never let vision override the gyro's rotation.
    kSingleTagStdDevs = (0.8, 0.8, 9999.0)
    kMultiTagStdDevs  = (0.3, 0.3, 9999.0)

    # Std devs scale up with distance² — inflates uncertainty for far/noisy tags.
    kDistanceScaleFactor = 0.08

    # Normal speed gate for pose injection.
    kMaxVisionSpeedMps = 2.5
    # Over the bump the robot moves fast — accept vision up to this speed but
    # multiply std devs so the correction is soft, not a sudden pose jump.
    kBumpCorrectionSpeedMps = 4.5
    kBumpStdDevMultiplier   = 3.0

    kMaxYawErrorDeg = 15.0

    # Hub AprilTag IDs for each alliance. All four Hub faces are tagged.
    # We filter by alliance to avoid accidentally using the far side's tags.
    # ⚠️ Confirm these IDs from the 2026 game manual / field layout JSON.
    kRedHubTagIds  = {2, 3, 4, 5, 8, 9, 10, 11}
    kBlueHubTagIds = {18, 19, 20, 21, 24, 25, 26, 27}


# ─────────────────────────────────────────────────────────────────────────────
# SHOOTING
# ─────────────────────────────────────────────────────────────────────────────
class ShootingConstants:
    # Intake, Limelight, and shooter all face the same direction (forward).
    # Robot drives forward toward the Hub; TX is used for alignment.
    # Distance is derived from robot pose → Hub tag position (NOT TY geometry,
    # because camera pitch is 0° and TY is unreliable at shallow angles).

    # ── Tune these on the physical robot ──────────────────────────
    # Horizontal distance from robot front to the Hub tag face at your chosen
    # shooting spot. Measure with a tape; affects both approach and flywheel.
    kTargetDistanceM = 2.0    # MEASURE THIS

    # Flywheel RPS at kTargetDistanceM. Start at 42 and adjust by watching shots.
    kTargetRps = 42.0         # TUNE THIS

    # ── Alignment tolerances ──────────────────────────────────────
    kTxToleranceDeg = 2.0     # how centered (in TX degrees) is "good enough"
    kDistToleranceM = 0.06    # ±6 cm

    # ── Proportional gains ────────────────────────────────────────
    # kAngleP: output = kAngleP × tx_degrees → rotation speed fraction
    #   Too high → oscillates. Too low → slow. Start at 0.03.
    kAngleP = 0.03            # TUNE THIS

    # kDistP: output = kDistP × dist_error_meters → forward speed fraction
    #   Too high → overshoots. Too low → creeps. Start at 0.40.
    kDistP  = 0.40            # TUNE THIS

    # ── Output clamps ─────────────────────────────────────────────
    kMaxAlignSpeed    = 0.35  # max rotation speed fraction
    kMaxApproachSpeed = 0.45  # max forward speed fraction

    # ── Settle / timeout ──────────────────────────────────────────
    # Both TX and distance must be within tolerance for this many consecutive
    # 20 ms loops (~100 ms) before the command declares "at position".
    kSettleCycles    = 5
    # Safety: if alignment takes longer than this, shoot anyway.
    kAlignTimeoutSec = 4.0


# ─────────────────────────────────────────────────────────────────────────────
# AUTONOMOUS (PathPlanner)
# ─────────────────────────────────────────────────────────────────────────────
class AutoConstants:
    kPxController     = 5.0
    kPyController     = 5.0
    kPThetaController = 5.0

    kMaxSpeedMps  = 4.8
    kMaxAccelMps2 = 5.0

    kRobotMassKg = 55.0
    kRobotMOI    = 4.5
    kWheelCOF    = 1.0


# ─────────────────────────────────────────────────────────────────────────────
# INTAKE DRAWER (disabled — subsystem is commented out)
# ─────────────────────────────────────────────────────────────────────────────
class IntakeDrawerConstants:
    kLeftDrawerMotorId  = 19
    kRightDrawerMotorId = 14
    
    k_config = SparkMaxConfig()

    k_config.inverted(True)
    k_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)
    k_config.smartCurrentLimit(40)

    k_follower_config = SparkMaxConfig()
    k_follower_config.follow(kLeftDrawerMotorId, invert=True)
    k_follower_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)