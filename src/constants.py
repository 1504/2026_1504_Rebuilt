"""
Team 1504 Desperate Penguins - Constants
2026 Season | FRC Game: Rebuilt
"""

import math
from wpimath.units import inchesToMeters, lbsToKilograms
from rev import ClosedLoopSlot, SparkClosedLoopController
import wpimath.units
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
    # Teleop top speed — intentionally conservative while tuning.
    # Raise toward kMaxAutoSpeedMps when you're confident in the robot.
    kMaxSpeedMps = 1.5

    # Physical ceiling used for PathPlanner desaturation and fallback config.
    # FIXED: auto was desaturating against kMaxSpeedMps (1.5 m/s), which made
    # PathPlanner's feedforward wrong and caused the robot to undershoot every
    # path segment.  Always desaturate auto against the real hardware limit.
    kMaxAutoSpeedMps = 4.8

    kMaxAngularSpeedRps = 2 * math.pi

    kMagnitudeSlewRate  = 3.0
    kRotationalSlewRate = 2.0
    kSlowModeMultiplier = 0.3

    kTrackWidth = 0.6155
    kWheelBase  = 0.6155

    kFrontLeftChassisOffset  = -math.pi / 2
    kFrontRightChassisOffset = 0.0
    kRearLeftChassisOffset   = math.pi
    kRearRightChassisOffset  = math.pi / 2

#change when you change motor

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

    k_start_x = 2.0   # meters from blue alliance wall
    k_start_y = 4.0   # meters from bottom wall (mid-field height)


# ─────────────────────────────────────────────────────────────────────────────
# SHOOTER  (replace the existing ShooterConstants class in constants.py)
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

    kVelocityToleranceRps = 2.0
    kFeederSpeed = -0.4
    kAgitatorSpeed = -0.15
    kFeederCurrentLimit = 20
    kFlywheelCurrentLimit = 60
    kFlywheelStatorCurrentLimit = 80

    # ── Live velocity tuning (D-pad) ──────────────────────────────
    kDefaultShooterRps = 10.0   # what "reset" returns to
    kShooterRpsStep    = 5.0    # how much each D-pad press changes velocity
    kShooterMinRps     = 10.0   # floor — won't go below this
    kShooterMaxRps     = 80.0   # ceiling — won't go above this

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

    kMotorId  = 9          # Leader  (SparkFlex / NEO Vortex)
    kMotorId2 = 15         # Follower (SparkFlex / NEO Vortex) — SET YOUR CAN ID
    kIntakeSpeed  =  0.15
    kReverseSpeed = -0.15
    kCurrentLimit = 70


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

    kCameraHeightMeters = 0.675
    kCameraPitch        = 0
    kCameraYaw          = 0.0

    kTagHeightMeters = 1.45

    kFieldMinX =  0.0
    kFieldMaxX = 16.54
    kFieldMinY =  0.0
    kFieldMaxY =  8.21

    kSingleTagStdDevs = (4.0, 4.0, 8.0)
    kMultiTagStdDevs  = (0.5, 0.5, 1.0)

    kDistanceScaleFactor = 0.10

    kMaxVisionSpeedMps = 3.0

    kMaxYawErrorDeg = 15.0


# ─────────────────────────────────────────────────────────────────────────────
# AUTONOMOUS  (PathPlanner)
# ─────────────────────────────────────────────────────────────────────────────
class AutoConstants:
    kPxController = 5.0
    kPyController = 5.0

    kPThetaController = 5.0

    # FIXED: was 1.5 (matching the teleop cap), so the fallback RobotConfig
    # told PathPlanner the robot could only do 1.5 m/s and all feedforward
    # was scaled wrong.  Match the PathPlanner GUI setting (4.8 m/s).
    kMaxSpeedMps   = 4.8
    kMaxAccelMps2  = 5.0

    kRobotMassKg = 55.0
    kRobotMOI    = 4.5

    kWheelCOF = 1.0


# ─────────────────────────────────────────────────────────────────────────────
# INTAKE Drawer
# ─────────────────────────────────────────────────────────────────────────────
class IntakeDrawerConstants:
    kLeftDrawerMotorId  = 19 #Motor is not conneted to a sparkmax.
    kRightDrawerMotorId = 14
    
    kDrawerStartPosition = 0
    kDrawerDeployedPosition = wpimath.units.inchesToMeters(6) 

    kDrawerEffectivePulleyRadius = 2
    k_max_acceleration_meter_per_sec_squared = 0.5
    k_max_velocity_meter_per_second = 0.5
    k_name = "intakeDrawer"

    k_kS_volts = 0 # constant to always add, uses the sign of velocity
    k_kG_volts = 0.88 / 2.0  # 12kg at .2m COM, cuts in half with two motors, goes up with mass and distance, down with efficiency
    k_kV_volt_second_per_radian = 12.05  # stays the same with one or two motors, based on the NEO itself and gear ratio
    k_kA_volt_second_squared_per_meter = 0.10 / 2.0 # cuts in half with 2 motors

    k_gear_ratio = 3 # 9, 12, or 15 gear ratio said victor 1/30/25
                      # we need it seperate for the sim
    k_effective_pulley_diameter = inchesToMeters(1.91) # (https://www.andymark.com/products/25-24-tooth-0-375-in-hex-sprocket) although we're using rev, rev doesn't give a pitch diameter
    k_meters_per_revolution = math.pi * 2 * k_effective_pulley_diameter / k_gear_ratio # 2 because our elevator goes twice as fast as the chain because continuous rigging

    k_config = SparkMaxConfig() 
    k_config.voltageCompensation(12)           
    k_config.inverted(True)

    k_config.encoder.positionConversionFactor(k_meters_per_revolution)
    k_config.encoder.velocityConversionFactor(k_meters_per_revolution / 60)

    k_config.closedLoop.pid(p=1.0, i=0, d=0, slot=ClosedLoopSlot(0))
    k_config.closedLoop.IZone(iZone=0, slot=ClosedLoopSlot(0))
    k_config.closedLoop.IMaxAccum(0, slot=ClosedLoopSlot(0))
    k_config.closedLoop.outputRange(-1, 1)

    k_config.softLimit.forwardSoftLimit(kDrawerDeployedPosition)
    k_config.softLimit.reverseSoftLimit(kDrawerStartPosition)

    k_config.softLimit.forwardSoftLimitEnabled(True)
    k_config.softLimit.reverseSoftLimitEnabled(True)

    k_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)
    k_config.smartCurrentLimit(40)

    k_follower_config = SparkMaxConfig()
    k_follower_config.follow(kRightDrawerMotorId, invert=True)
    k_follower_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)